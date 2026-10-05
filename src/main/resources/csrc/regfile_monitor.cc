// DPI-C sink for boom.monitors.RegFileMonitor.
//
// Writes one binary file per monitored register file:
//
//   Header (32 bytes, little-endian):
//     0  u32  magic "RGMN"
//     4  u16  version (1)
//     6  u8   counter_bytes (1, 2 or 4)
//     7  u8   rf_type (0 = int, 1 = fp)
//     8  u32  num_regs
//    12  u32  hart_id
//    16  u64  window_cycles
//    24  u32  num_windows        (0xFFFFFFFF if the file was not closed cleanly)
//    28  u32  last_window_cycles (cycles covered by the last window; < window_cycles
//                                 when it is the trailing partial window)
//   Then num_windows records, each:
//     reads [num_regs]  counter_bytes each
//     writes[num_regs]  counter_bytes each
//
// Records arrive one register per cycle in preg order; a window is buffered
// until complete and appended to a large output buffer that is written with
// a single fwrite when full.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kMagic        = 0x4E4D4752; // "RGMN"
constexpr uint16_t kVersion      = 1;
constexpr size_t   kHeaderBytes  = 32;
constexpr size_t   kOutBufBytes  = 4 << 20;
constexpr uint32_t kUnknownCount = 0xFFFFFFFF;

struct RegMon {
  FILE*                f = nullptr;
  std::string          path;
  uint32_t             num_regs = 0;
  uint32_t             cbytes = 0;
  uint64_t             window_cycles = 0;
  std::vector<uint8_t> win;    // window being assembled: reads then writes
  std::vector<uint8_t> live;   // trailing partial window (final block only)
  std::vector<uint8_t> out;    // output buffer
  uint32_t             filled = 0;
  uint32_t             num_windows = 0;
  uint32_t             last_window_cycles = 0;
  bool                 final_pending = false;
  uint32_t             final_filled = 0;

  void put(std::vector<uint8_t>& buf, uint32_t preg, uint32_t rd, uint32_t wr) {
    if (preg >= num_regs) return;
    // little-endian host: low bytes first
    std::memcpy(&buf[preg * cbytes], &rd, cbytes);
    std::memcpy(&buf[(num_regs + preg) * cbytes], &wr, cbytes);
  }

  void flush_out() {
    if (!out.empty()) {
      std::fwrite(out.data(), 1, out.size(), f);
      out.clear();
    }
  }

  void commit(const std::vector<uint8_t>& buf, uint32_t cycles) {
    out.insert(out.end(), buf.begin(), buf.end());
    num_windows++;
    last_window_cycles = cycles;
    if (out.size() >= kOutBufBytes) flush_out();
  }

  void write_counts(uint32_t windows, uint32_t last_cycles) {
    std::fseek(f, 24, SEEK_SET);
    std::fwrite(&windows, 4, 1, f);
    std::fwrite(&last_cycles, 4, 1, f);
  }

  void close() {
    if (!f) return;
    flush_out();
    write_counts(num_windows, last_window_cycles);
    std::fclose(f);
    f = nullptr;
  }
};

std::vector<RegMon*> g_open;

// Flush files whose final block never ran (e.g. $fatal or a timeout).
// Buffered complete windows are kept; partial data is dropped.
void close_all_at_exit() {
  for (RegMon* m : g_open) {
    if (m->f) {
      std::fprintf(stderr, "[regmon] %s: closed at exit without final block (%u windows)\n",
                   m->path.c_str(), m->num_windows);
      m->close();
    }
  }
}

uint32_t bytes_for(int bits) {
  if (bits <= 8)  return 1;
  if (bits <= 16) return 2;
  return 4;
}

} // namespace

extern "C" void* regmon_open(const char* prefix, const char* name, int rf_type, int hart_id,
                             int num_regs, int counter_bits, long long window_cycles)
{
  RegMon* m = new RegMon;
  m->path          = std::string(prefix) + "_" + name + "_hart" + std::to_string(hart_id) + ".bin";
  m->num_regs      = num_regs;
  m->cbytes        = bytes_for(counter_bits);
  m->window_cycles = window_cycles;
  m->win.assign(2 * num_regs * m->cbytes, 0);
  m->out.reserve(kOutBufBytes + m->win.size());

  m->f = std::fopen(m->path.c_str(), "wb");
  if (!m->f) {
    std::fprintf(stderr, "[regmon] cannot open %s, logging disabled\n", m->path.c_str());
    delete m;
    return nullptr;
  }

  uint8_t hdr[kHeaderBytes] = {0};
  uint16_t version = kVersion;
  uint8_t  cbytes  = m->cbytes;
  uint8_t  rtype   = rf_type;
  uint32_t nregs   = num_regs;
  uint32_t hart    = hart_id;
  uint64_t wcycles = window_cycles;
  uint32_t unknown = kUnknownCount;
  std::memcpy(hdr + 0,  &kMagic,  4);
  std::memcpy(hdr + 4,  &version, 2);
  std::memcpy(hdr + 6,  &cbytes,  1);
  std::memcpy(hdr + 7,  &rtype,   1);
  std::memcpy(hdr + 8,  &nregs,   4);
  std::memcpy(hdr + 12, &hart,    4);
  std::memcpy(hdr + 16, &wcycles, 8);
  std::memcpy(hdr + 24, &unknown, 4);
  std::fwrite(hdr, 1, kHeaderBytes, m->f);

  if (g_open.empty()) std::atexit(close_all_at_exit);
  g_open.push_back(m);
  return m;
}

extern "C" void regmon_record(void* h, int preg, int reads, int writes)
{
  RegMon* m = static_cast<RegMon*>(h);
  m->put(m->win, preg, reads, writes);
  if (++m->filled == m->num_regs) {
    m->commit(m->win, m->window_cycles);
    m->filled = 0;
  }
}

// Final block, step 1: a window is still being streamed if some of its records
// arrived or the hardware is still dumping; the rest comes from the snapshot.
extern "C" void regmon_final_begin(void* h, int dumping)
{
  RegMon* m = static_cast<RegMon*>(h);
  m->final_pending = m->filled > 0 || dumping;
  m->final_filled  = m->filled;
  m->live.assign(m->win.size(), 0);
}

extern "C" void regmon_final_snap(void* h, int preg, int reads, int writes)
{
  RegMon* m = static_cast<RegMon*>(h);
  if (!m->final_pending) return;
  if ((uint32_t)preg >= m->final_filled) m->put(m->win, preg, reads, writes);
  if ((uint32_t)preg == m->num_regs - 1) {
    m->commit(m->win, m->window_cycles);
    m->filled = 0;
    m->final_pending = false;
  }
}

extern "C" void regmon_final_live(void* h, int preg, int reads, int writes)
{
  RegMon* m = static_cast<RegMon*>(h);
  m->put(m->live, preg, reads, writes);
}

// Final block, last step: append the trailing partial window and close.
extern "C" void regmon_close(void* h, int window_cycle)
{
  RegMon* m = static_cast<RegMon*>(h);
  if (window_cycle > 0) m->commit(m->live, window_cycle);
  m->close();
}
