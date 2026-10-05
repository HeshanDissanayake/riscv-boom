// DPI-C sink for boom.monitors.MonitorLogger (shared by all monitors).
//
// Writes one binary file per monitor instance, <prefix>_<tag>_hart<N>.bin:
//
//   Header (40 bytes, little-endian):
//     0  u32  magic "BMON"
//     4  u16  version (1)
//     6  u8   counter_bytes (1, 2 or 4)
//     7  u8   reserved (0)
//     8  u32  num_fields
//    12  u32  num_rows
//    16  u64  window_cycles
//    24  u32  num_windows        (0xFFFFFFFF if the file was not closed cleanly)
//    28  u32  last_window_cycles (cycles covered by the last window; < window_cycles
//                                 when it is the trailing partial window)
//    32  u32  hart_id
//    36  u32  meta_bytes
//   Then meta_bytes of JSON metadata (space padded to a multiple of 8):
//     {"kind": ..., "name": ..., "fields": [...], "row_names": [...]}
//   Then num_windows records, each field-major:
//     field 0: values[num_rows]  counter_bytes each
//     field 1: values[num_rows]
//     ...
//
// Rows arrive one per cycle in row order; a window is buffered until complete
// and appended to a large output buffer that is written with a single fwrite
// when full.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kMagic        = 0x4E4F4D42; // "BMON"
constexpr uint16_t kVersion      = 1;
constexpr size_t   kHeaderBytes  = 40;
constexpr size_t   kOutBufBytes  = 4 << 20;
constexpr uint32_t kUnknownCount = 0xFFFFFFFF;

struct MonLog {
  FILE*                f = nullptr;
  std::string          path;
  uint32_t             num_fields = 0;
  uint32_t             num_rows = 0;
  uint32_t             cbytes = 0;
  uint64_t             window_cycles = 0;
  std::vector<uint8_t> win;    // window being assembled
  std::vector<uint8_t> live;   // trailing partial window (final block only)
  std::vector<uint8_t> out;    // output buffer
  uint32_t             filled = 0;   // rows of `win` received
  uint32_t             num_windows = 0;
  uint32_t             last_window_cycles = 0;
  bool                 final_pending = false;
  uint32_t             final_filled = 0;

  void put(std::vector<uint8_t>& buf, uint32_t row, uint32_t field, uint32_t v) {
    if (row >= num_rows || field >= num_fields) return;
    // little-endian host: low bytes first
    std::memcpy(&buf[(field * num_rows + row) * cbytes], &v, cbytes);
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

std::vector<MonLog*> g_open;

// Flush files whose final block never ran (e.g. $fatal or a timeout).
// Buffered complete windows are kept; partial data is dropped.
void close_all_at_exit() {
  for (MonLog* m : g_open) {
    if (m->f) {
      std::fprintf(stderr, "[monlog] %s: closed at exit without final block (%u windows)\n",
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

MonLog* as_log(void* h) { return static_cast<MonLog*>(h); }

} // namespace

extern "C" void* monlog_open(const char* prefix, const char* tag, const char* meta, int hart_id,
                             int num_fields, int num_rows, int counter_bits,
                             long long window_cycles)
{
  MonLog* m = new MonLog;
  m->path          = std::string(prefix) + "_" + tag + "_hart" + std::to_string(hart_id) + ".bin";
  m->num_fields    = num_fields;
  m->num_rows      = num_rows;
  m->cbytes        = bytes_for(counter_bits);
  m->window_cycles = window_cycles;
  m->win.assign((size_t)num_fields * num_rows * m->cbytes, 0);
  m->out.reserve(kOutBufBytes + m->win.size());

  m->f = std::fopen(m->path.c_str(), "wb");
  if (!m->f) {
    std::fprintf(stderr, "[monlog] cannot open %s, logging disabled\n", m->path.c_str());
    delete m;
    return nullptr;
  }

  std::string json(meta);
  json.append((8 - json.size() % 8) % 8, ' ');

  uint8_t  hdr[kHeaderBytes] = {0};
  uint16_t version = kVersion;
  uint8_t  cbytes  = m->cbytes;
  uint32_t nfields = num_fields;
  uint32_t nrows   = num_rows;
  uint64_t wcycles = window_cycles;
  uint32_t unknown = kUnknownCount;
  uint32_t hart    = hart_id;
  uint32_t mbytes  = json.size();
  std::memcpy(hdr + 0,  &kMagic,  4);
  std::memcpy(hdr + 4,  &version, 2);
  std::memcpy(hdr + 6,  &cbytes,  1);
  std::memcpy(hdr + 8,  &nfields, 4);
  std::memcpy(hdr + 12, &nrows,   4);
  std::memcpy(hdr + 16, &wcycles, 8);
  std::memcpy(hdr + 24, &unknown, 4);
  std::memcpy(hdr + 32, &hart,    4);
  std::memcpy(hdr + 36, &mbytes,  4);
  std::fwrite(hdr, 1, kHeaderBytes, m->f);
  std::fwrite(json.data(), 1, json.size(), m->f);

  if (g_open.empty()) std::atexit(close_all_at_exit);
  g_open.push_back(m);
  return m;
}

extern "C" void monlog_value(void* h, int row, int field, int value)
{
  MonLog* m = as_log(h);
  m->put(m->win, row, field, value);
}

extern "C" void monlog_row_done(void* h)
{
  MonLog* m = as_log(h);
  if (++m->filled == m->num_rows) {
    m->commit(m->win, m->window_cycles);
    m->filled = 0;
  }
}

// Final block, step 1: a window is still being streamed if some of its rows
// arrived or the hardware is still dumping; the rest comes from the snapshot.
extern "C" void monlog_final_begin(void* h, int dumping)
{
  MonLog* m = as_log(h);
  m->final_pending = m->filled > 0 || dumping;
  m->final_filled  = m->filled;
  m->live.assign(m->win.size(), 0);
}

extern "C" void monlog_final_snap(void* h, int row, int field, int value)
{
  MonLog* m = as_log(h);
  if (m->final_pending && (uint32_t)row >= m->final_filled) m->put(m->win, row, field, value);
}

extern "C" void monlog_final_snap_done(void* h)
{
  MonLog* m = as_log(h);
  if (!m->final_pending) return;
  m->commit(m->win, m->window_cycles);
  m->filled = 0;
  m->final_pending = false;
}

extern "C" void monlog_final_live(void* h, int row, int field, int value)
{
  MonLog* m = as_log(h);
  m->put(m->live, row, field, value);
}

// Final block, last step: append the trailing partial window and close.
extern "C" void monlog_close(void* h, int window_cycle)
{
  MonLog* m = as_log(h);
  if (window_cycle > 0) m->commit(m->live, window_cycle);
  m->close();
}
