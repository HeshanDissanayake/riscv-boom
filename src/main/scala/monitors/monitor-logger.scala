//******************************************************************************
// Monitor Logger
//------------------------------------------------------------------------------
//
// Shared windowing and output plumbing for the passive monitors in this
// package. A monitor keeps its own live counters; the logger
//  - times fixed windows of `windowCycles` cycles (io.windowEnd is high in the
//    last cycle of each window, when the monitor must clear its counters),
//  - snapshots io.snap at the end of each window,
//  - streams the snapshot out on io.dump, one row per cycle, and
//  - optionally writes the stream to a binary file through DPI-C.
//
// Counters are organised as a table of `fields` x `numRows` (e.g. for the
// register file monitor: fields reads/writes, one row per physical register).
//
// Every logger counts cycles from reset, so all monitors in a design share
// the same window boundaries.
//
// Output sinks (RTL simulation only):
//  - dpiLog:    MonitorLogDPI writes a binary file (format in
//               csrc/monitor_log.cc). At the end of simulation it also
//               writes the trailing partial window from io.live.
//  - printDump: printf each dumped row (debug).
// Both must be off for FPGA (FireSim) builds.

package boom.monitors

import chisel3._
import chisel3.util._

/**
 * Common parameters for monitors built on MonitorLogger.
 *
 * @param windowCycles number of cycles per sampling window
 * @param dpiLog write the dump stream to a binary file through DPI-C
 * @param printDump print each dumped row with printf
 */
case class MonitorParams(
  windowCycles: Int = 10000,
  dpiLog: Boolean = true,
  printDump: Boolean = false)

/**
 * Static description of a monitor's log.
 *
 * @param kind monitor type ("regfile", ...), recorded in the log header
 * @param tag instance name; output file is <prefix>_<tag>_hart<N>.bin
 * @param hartId static tile id
 * @param fields counter names, one per column
 * @param numRows number of rows of counters
 * @param rowNames optional row labels, recorded in the log header
 * @param extra additional integer metadata, recorded in the log header
 */
case class MonitorLogInfo(
  kind: String,
  tag: String,
  hartId: Int,
  fields: Seq[String],
  numRows: Int,
  rowNames: Seq[String] = Nil,
  extra: Seq[(String, BigInt)] = Nil)
{
  require(fields.nonEmpty && numRows >= 1)
  require(rowNames.isEmpty || rowNames.length == numRows)

  def numFields: Int = fields.length

  /** Header metadata as a JSON object. */
  def json: String = {
    def str(s: String) = "\"" + s.flatMap {
      case '"'  => "\\\""
      case '\\' => "\\\\"
      case c    => c.toString
    } + "\""
    def arr(xs: Seq[String]) = xs.map(str).mkString("[", ",", "]")
    val ext = extra.map { case (k, v) => s",${str(k)}:$v" }.mkString
    s"""{"kind":${str(kind)},"name":${str(tag)},"fields":${arr(fields)},"row_names":${arr(rowNames)}$ext}"""
  }
}

class MonitorRecord(val numRows: Int, val numFields: Int, val counterBits: Int) extends Bundle
{
  val window = UInt(32.W)
  val row    = UInt(log2Ceil(numRows max 2).W)
  val values = Vec(numFields, UInt(counterBits.W))
}

/**
 * @param counterBits width of every counter
 * @param windowCycles number of cycles per window
 * @param dpiLog write the dump stream to a binary file through DPI-C
 * @param printDump print each dumped row with printf
 */
class MonitorLogger(
  val info: MonitorLogInfo,
  counterBits: Int,
  windowCycles: Int,
  dpiLog: Boolean,
  printDump: Boolean) extends Module
{
  private val F = info.numFields
  private val R = info.numRows

  // The snapshot must be fully streamed out before the next window ends.
  require(windowCycles > R,
    s"MonitorLogger(${info.tag}): windowCycles ($windowCycles) must exceed numRows ($R)")
  require(!dpiLog || counterBits <= 32,
    s"MonitorLogger(${info.tag}): dpiLog supports at most 32-bit counters")

  val io = IO(new Bundle {
    val windowEnd = Output(Bool())
    // value of each counter including this cycle's events; sampled when windowEnd
    val snap      = Input(Vec(F, Vec(R, UInt(counterBits.W))))
    // live counters (excluding this cycle); only read for the trailing partial window
    val live      = Input(Vec(F, Vec(R, UInt(counterBits.W))))
    val dump      = Output(Valid(new MonitorRecord(R, F, counterBits)))
  })

  // --------------------------------------------------------------
  // Window timing

  val cycle     = RegInit(0.U(log2Ceil(windowCycles).W))
  val windowEnd = cycle === (windowCycles - 1).U
  cycle := Mux(windowEnd, 0.U, cycle + 1.U)
  io.windowEnd := windowEnd

  val windowId = RegInit(0.U(32.W))

  // --------------------------------------------------------------
  // Snapshot and stream it out, one row per cycle

  val snap = Reg(Vec(F, Vec(R, UInt(counterBits.W))))
  when (windowEnd) { snap := io.snap }

  val dumping    = RegInit(false.B)
  val dumpIdx    = RegInit(0.U(log2Ceil(R max 2).W))
  val dumpWindow = RegInit(0.U(32.W))

  when (windowEnd) {
    dumping    := true.B
    dumpIdx    := 0.U
    dumpWindow := windowId
    windowId   := windowId + 1.U
  } .elsewhen (dumping) {
    dumpIdx := dumpIdx + 1.U
    when (dumpIdx === (R - 1).U) {
      dumping := false.B
    }
  }

  io.dump.valid       := dumping
  io.dump.bits.window := dumpWindow
  io.dump.bits.row    := dumpIdx
  io.dump.bits.values := VecInit(snap.map(_(dumpIdx)))

  // --------------------------------------------------------------
  // Sinks

  if (dpiLog) {
    val dpi = Module(new MonitorLogDPI(info, counterBits, windowCycles))
    dpi.io.clock        := clock
    dpi.io.reset        := reset.asBool
    dpi.io.dump_valid   := io.dump.valid
    dpi.io.dump_row     := io.dump.bits.row
    dpi.io.dump_values  := io.dump.bits.values.asUInt
    dpi.io.dumping      := dumping
    dpi.io.window_cycle := cycle
    dpi.io.snap         := snap.asUInt
    dpi.io.live         := io.live.asUInt
  }

  if (printDump) {
    when (io.dump.valid) {
      val fmt = info.fields.map(f => s" $f=%d").mkString
      printf(Printable.pack(s"[mon-${info.tag}] window=%d row=%d$fmt\n",
        (Seq(io.dump.bits.window, io.dump.bits.row) ++ io.dump.bits.values): _*))
    }
  }
}

/**
 * DPI-C sink for MonitorLogger. Writes each dumped row to a binary file and,
 * in a `final` block, completes any window still being streamed and appends
 * the trailing partial window from the live counters.
 *
 * Wide buses carry the counter table flattened field-major: counter
 * (field f, row r) is at index f*numRows + r.
 *
 * The Verilog is generated per instance so each gets a unique module name
 * with its widths baked in. Plusargs (shared by all monitors):
 *   +regmon_prefix=<path>  output file prefix (default "regmon")
 *   +regmon_off            disable file output
 */
class MonitorLogDPI(
  info: MonitorLogInfo,
  counterBits: Int,
  windowCycles: Int) extends BlackBox with HasBlackBoxInline with HasBlackBoxResource
{
  private val F         = info.numFields
  private val R         = info.numRows
  private val W         = counterBits
  private val rowBits   = log2Ceil(R max 2)
  private val cycleBits = log2Ceil(windowCycles)

  val io = IO(new Bundle {
    val clock        = Input(Clock())
    val reset        = Input(Bool())
    val dump_valid   = Input(Bool())
    val dump_row     = Input(UInt(rowBits.W))
    val dump_values  = Input(UInt((F * W).W))
    val dumping      = Input(Bool())
    val window_cycle = Input(UInt(cycleBits.W))
    val snap         = Input(UInt((F * R * W).W))
    val live         = Input(UInt((F * R * W).W))
  })

  override def desiredName = s"MonitorLogDPI_${info.tag}_hart${info.hartId}"

  addResource("/csrc/monitor_log.cc")

  // JSON as a Verilog string literal
  private val meta = info.json.flatMap {
    case '"'  => "\\\""
    case '\\' => "\\\\"
    case c    => c.toString
  }

  setInline(s"$desiredName.v",
    s"""module $desiredName (
       |  input                       clock,
       |  input                       reset,
       |  input                       dump_valid,
       |  input  [${rowBits-1}:0]     dump_row,
       |  input  [${F*W-1}:0]         dump_values,
       |  input                       dumping,
       |  input  [${cycleBits-1}:0]   window_cycle,
       |  input  [${F*R*W-1}:0]       snap,
       |  input  [${F*R*W-1}:0]       live
       |);
       |  import "DPI-C" function chandle monlog_open(input string prefix, input string tag,
       |    input string meta, input int hart_id, input int num_fields, input int num_rows,
       |    input int counter_bits, input longint window_cycles);
       |  import "DPI-C" function void monlog_value(input chandle h, input int row,
       |    input int field, input int value);
       |  import "DPI-C" function void monlog_row_done(input chandle h);
       |  import "DPI-C" function void monlog_final_begin(input chandle h, input int dumping);
       |  import "DPI-C" function void monlog_final_snap(input chandle h, input int row,
       |    input int field, input int value);
       |  import "DPI-C" function void monlog_final_snap_done(input chandle h);
       |  import "DPI-C" function void monlog_final_live(input chandle h, input int row,
       |    input int field, input int value);
       |  import "DPI-C" function void monlog_close(input chandle h, input int window_cycle);
       |
       |  localparam W = $W;
       |  localparam F = $F;
       |  localparam R = $R;
       |
       |  chandle h;
       |  string  prefix;
       |  integer f, i;
       |
       |  initial begin
       |    h = null;
       |    if (!$$test$$plusargs("regmon_off")) begin
       |      if (!$$value$$plusargs("regmon_prefix=%s", prefix)) prefix = "regmon";
       |      h = monlog_open(prefix, "${info.tag}", "$meta", ${info.hartId}, F, R, W, ${windowCycles});
       |    end
       |  end
       |
       |  always @(posedge clock) begin
       |    if (!reset && dump_valid && h != null) begin
       |      for (f = 0; f < F; f = f + 1)
       |        monlog_value(h, int'(dump_row), f, int'(dump_values[f*W +: W]));
       |      monlog_row_done(h);
       |    end
       |  end
       |
       |  final begin
       |    if (h != null) begin
       |      monlog_final_begin(h, int'(dumping));
       |      for (i = 0; i < F*R; i = i + 1)
       |        monlog_final_snap(h, i % R, i / R, int'(snap[i*W +: W]));
       |      monlog_final_snap_done(h);
       |      for (i = 0; i < F*R; i = i + 1)
       |        monlog_final_live(h, i % R, i / R, int'(live[i*W +: W]));
       |      monlog_close(h, int'(window_cycle));
       |    end
       |  end
       |endmodule
       |""".stripMargin)
}
