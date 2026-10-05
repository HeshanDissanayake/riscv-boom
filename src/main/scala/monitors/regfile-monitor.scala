//******************************************************************************
// Register File Usage Monitor
//------------------------------------------------------------------------------
//
// A passive observer that counts, per physical register, how many times it is
// read and written. Counts are collected over fixed windows of `windowCycles`
// cycles; at the end of each window the counts are snapshotted, the live
// counters reset, and the snapshot is streamed out on `io.dump` one register
// per cycle as (window, preg, reads, writes).
//
// The monitor does not modify the register file or the register-read stage.
// It is instantiated by the parent (core / fp-pipeline) only when
// BoomCoreParams.regfileMonitor is set (see WithRegFileMonitor).
//
// Read events are reconstructed from the issued micro-ops that drive the
// register-read stage: operand k of issue slot w is a real read when the slot
// is valid and the operand's register type matches the register file. The
// register file registers its read address, so read events are delayed one
// cycle to count them in the cycle the array is actually accessed.
//
// Output sinks (RTL simulation only):
//  - dpiLog:    a DPI-C BlackBox writes the dump stream to a binary file
//               (see csrc/regfile_monitor.cc for the format). At the end of
//               simulation it also writes the trailing partial window.
//  - printDump: printf each record (debug).
// Both must be off for FPGA (FireSim) builds.

package boom.monitors

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Parameters, Config}
import freechips.rocketchip.subsystem.{TilesLocated, InSubsystem}

import boom.common._
import boom.exu.{RegisterFileWritePort}

/**
 * @param windowCycles number of cycles per sampling window
 * @param counterBits width of each read/write counter; derived from the
 *                    largest possible per-window count when not given
 * @param dpiLog write the dump stream to a binary file through DPI-C
 * @param printDump print each dumped record with printf
 */
case class RegFileMonitorParams(
  windowCycles: Int = 10000,
  counterBits: Option[Int] = None,
  dpiLog: Boolean = true,
  printDump: Boolean = false)

class RegFileMonitorRecord(val numRegs: Int, val counterBits: Int) extends Bundle
{
  val window = UInt(32.W)
  val preg   = UInt(log2Ceil(numRegs).W)
  val reads  = UInt(counterBits.W)
  val writes = UInt(counterBits.W)
}

/**
 * @param name register file tag ("int" / "fp")
 * @param hartId static tile id, used to name output files
 * @param numRegs number of physical registers
 * @param numReadPorts number of read events per cycle
 * @param numWritePorts number of write events per cycle
 * @param addrWidth width of the physical register address
 */
class RegFileMonitor(
  name: String,
  hartId: Int,
  numRegs: Int,
  numReadPorts: Int,
  numWritePorts: Int,
  addrWidth: Int,
  params: RegFileMonitorParams) extends Module
{
  private val windowCycles = params.windowCycles

  // Every port can hit the same register in a cycle.
  private val maxCount    = BigInt(windowCycles) * (numReadPorts max numWritePorts)
  val counterBits: Int    = params.counterBits.getOrElse(log2Ceil(maxCount + 1))

  // The snapshot must be fully streamed out before the next window ends.
  require(windowCycles > numRegs,
    s"RegFileMonitor($name): windowCycles ($windowCycles) must exceed numRegs ($numRegs)")
  require(maxCount < (BigInt(1) << counterBits),
    s"RegFileMonitor($name): counterBits ($counterBits) too small for windowCycles ($windowCycles)")
  require(!params.dpiLog || counterBits <= 32,
    s"RegFileMonitor($name): dpiLog supports at most 32-bit counters")

  val io = IO(new Bundle {
    val reads  = Input(Vec(numReadPorts, Valid(UInt(addrWidth.W))))
    val writes = Input(Vec(numWritePorts, Valid(UInt(addrWidth.W))))
    val dump   = Output(Valid(new RegFileMonitorRecord(numRegs, counterBits)))
  })

  // --------------------------------------------------------------
  // Window timing

  val cycle     = RegInit(0.U(log2Ceil(windowCycles).W))
  val windowEnd = cycle === (windowCycles - 1).U
  cycle := Mux(windowEnd, 0.U, cycle + 1.U)

  val windowId = RegInit(0.U(32.W))

  // --------------------------------------------------------------
  // Live counters and end-of-window snapshot

  val rdCnt  = RegInit(VecInit(Seq.fill(numRegs)(0.U(counterBits.W))))
  val wrCnt  = RegInit(VecInit(Seq.fill(numRegs)(0.U(counterBits.W))))
  val rdSnap = Reg(Vec(numRegs, UInt(counterBits.W)))
  val wrSnap = Reg(Vec(numRegs, UInt(counterBits.W)))

  for (r <- 0 until numRegs) {
    val rdHits = PopCount(io.reads.map(e => e.valid && e.bits === r.U))
    val wrHits = PopCount(io.writes.map(e => e.valid && e.bits === r.U))
    val rdNext = rdCnt(r) + rdHits
    val wrNext = wrCnt(r) + wrHits

    // Events in the last cycle of a window belong to that window.
    rdCnt(r) := Mux(windowEnd, 0.U, rdNext)
    wrCnt(r) := Mux(windowEnd, 0.U, wrNext)
    when (windowEnd) {
      rdSnap(r) := rdNext
      wrSnap(r) := wrNext
    }
  }

  // --------------------------------------------------------------
  // Stream the snapshot out, one register per cycle

  val dumping    = RegInit(false.B)
  val dumpIdx    = RegInit(0.U(log2Ceil(numRegs).W))
  val dumpWindow = RegInit(0.U(32.W))

  when (windowEnd) {
    dumping    := true.B
    dumpIdx    := 0.U
    dumpWindow := windowId
    windowId   := windowId + 1.U
  } .elsewhen (dumping) {
    dumpIdx := dumpIdx + 1.U
    when (dumpIdx === (numRegs - 1).U) {
      dumping := false.B
    }
  }

  io.dump.valid       := dumping
  io.dump.bits.window := dumpWindow
  io.dump.bits.preg   := dumpIdx
  io.dump.bits.reads  := rdSnap(dumpIdx)
  io.dump.bits.writes := wrSnap(dumpIdx)

  // --------------------------------------------------------------
  // Sinks

  if (params.dpiLog) {
    val rfType = name match {
      case "int" => 0
      case "fp"  => 1
      case _     => throw new IllegalArgumentException(s"RegFileMonitor: unknown register file '$name'")
    }
    val dpi = Module(new RegFileMonitorDPI(name, rfType, hartId, numRegs, counterBits, windowCycles))
    dpi.io.clock        := clock
    dpi.io.reset        := reset.asBool
    dpi.io.dump_valid   := io.dump.valid
    dpi.io.dump_preg    := io.dump.bits.preg
    dpi.io.dump_reads   := io.dump.bits.reads
    dpi.io.dump_writes  := io.dump.bits.writes
    dpi.io.dumping      := dumping
    dpi.io.window_cycle := cycle
    dpi.io.live_reads   := rdCnt.asUInt
    dpi.io.live_writes  := wrCnt.asUInt
    dpi.io.snap_reads   := rdSnap.asUInt
    dpi.io.snap_writes  := wrSnap.asUInt
  }

  if (params.printDump) {
    when (io.dump.valid) {
      printf(s"[regmon-$name] window=%d preg=%d reads=%d writes=%d\n",
        io.dump.bits.window, io.dump.bits.preg, io.dump.bits.reads, io.dump.bits.writes)
    }
  }
}

/**
 * DPI-C sink for RegFileMonitor. Writes each dumped record to a binary file
 * and, in a `final` block, completes any window still being streamed and
 * appends the trailing partial window from the live counters.
 *
 * The Verilog is generated per instance so each gets a unique module name
 * with its widths baked in. Plusargs:
 *   +regmon_prefix=<path>  output file prefix (default "regmon"); files are
 *                          <prefix>_<int|fp>_hart<N>.bin
 *   +regmon_off            disable file output
 */
class RegFileMonitorDPI(
  name: String,
  rfType: Int,
  hartId: Int,
  numRegs: Int,
  counterBits: Int,
  windowCycles: Int) extends BlackBox with HasBlackBoxInline with HasBlackBoxResource
{
  private val pregBits  = log2Ceil(numRegs)
  private val wideBits  = numRegs * counterBits
  private val cycleBits = log2Ceil(windowCycles)

  val io = IO(new Bundle {
    val clock        = Input(Clock())
    val reset        = Input(Bool())
    val dump_valid   = Input(Bool())
    val dump_preg    = Input(UInt(pregBits.W))
    val dump_reads   = Input(UInt(counterBits.W))
    val dump_writes  = Input(UInt(counterBits.W))
    val dumping      = Input(Bool())
    val window_cycle = Input(UInt(cycleBits.W))
    val live_reads   = Input(UInt(wideBits.W))
    val live_writes  = Input(UInt(wideBits.W))
    val snap_reads   = Input(UInt(wideBits.W))
    val snap_writes  = Input(UInt(wideBits.W))
  })

  override def desiredName = s"RegFileMonitorDPI_${name}_hart${hartId}"

  addResource("/csrc/regfile_monitor.cc")

  setInline(s"$desiredName.v",
    s"""module $desiredName (
       |  input                     clock,
       |  input                     reset,
       |  input                     dump_valid,
       |  input  [${pregBits-1}:0]  dump_preg,
       |  input  [${counterBits-1}:0] dump_reads,
       |  input  [${counterBits-1}:0] dump_writes,
       |  input                     dumping,
       |  input  [${cycleBits-1}:0] window_cycle,
       |  input  [${wideBits-1}:0]  live_reads,
       |  input  [${wideBits-1}:0]  live_writes,
       |  input  [${wideBits-1}:0]  snap_reads,
       |  input  [${wideBits-1}:0]  snap_writes
       |);
       |  import "DPI-C" function chandle regmon_open(input string prefix, input string name,
       |    input int rf_type, input int hart_id, input int num_regs, input int counter_bits,
       |    input longint window_cycles);
       |  import "DPI-C" function void regmon_record(input chandle h, input int preg,
       |    input int reads, input int writes);
       |  import "DPI-C" function void regmon_final_begin(input chandle h, input int dumping);
       |  import "DPI-C" function void regmon_final_snap(input chandle h, input int preg,
       |    input int reads, input int writes);
       |  import "DPI-C" function void regmon_final_live(input chandle h, input int preg,
       |    input int reads, input int writes);
       |  import "DPI-C" function void regmon_close(input chandle h, input int window_cycle);
       |
       |  localparam W = $counterBits;
       |  localparam N = $numRegs;
       |
       |  chandle h;
       |  string  prefix;
       |  integer i;
       |
       |  initial begin
       |    h = null;
       |    if (!$$test$$plusargs("regmon_off")) begin
       |      if (!$$value$$plusargs("regmon_prefix=%s", prefix)) prefix = "regmon";
       |      h = regmon_open(prefix, "$name", $rfType, $hartId, N, W, ${windowCycles});
       |    end
       |  end
       |
       |  always @(posedge clock) begin
       |    if (!reset && dump_valid && h != null)
       |      regmon_record(h, int'(dump_preg), int'(dump_reads), int'(dump_writes));
       |  end
       |
       |  final begin
       |    if (h != null) begin
       |      regmon_final_begin(h, int'(dumping));
       |      for (i = 0; i < N; i = i + 1)
       |        regmon_final_snap(h, i, int'(snap_reads[i*W +: W]), int'(snap_writes[i*W +: W]));
       |      for (i = 0; i < N; i = i + 1)
       |        regmon_final_live(h, i, int'(live_reads[i*W +: W]), int'(live_writes[i*W +: W]));
       |      regmon_close(h, int'(window_cycle));
       |    end
       |  end
       |endmodule
       |""".stripMargin)
}

object RegFileMonitor
{
  /**
   * Build a monitor that observes a register file from its parent.
   *
   * @param hartId static tile id, used to name output files
   * @param issValids issue-slot valids feeding the register-read stage
   * @param issUops issued micro-ops feeding the register-read stage
   * @param portsPerSlot read ports per issue slot (operands rs1, rs2, rs3 in order)
   * @param rtype register type of this register file (RT_FIX / RT_FLT)
   * @param writePorts the register file's write ports
   */
  def apply(
    name: String,
    hartId: Int,
    numRegs: Int,
    issValids: Seq[Bool],
    issUops: Seq[MicroOp],
    portsPerSlot: Int,
    rtype: UInt,
    writePorts: Seq[Valid[RegisterFileWritePort]],
    params: RegFileMonitorParams): RegFileMonitor =
  {
    require(issValids.length == issUops.length)
    require(portsPerSlot >= 1 && portsPerSlot <= 3)

    val addrWidth = writePorts.head.bits.addr.getWidth

    // (valid, addr) per read port, in the same order as the register-read stage
    val readEvents = (issValids zip issUops).flatMap { case (v, u) =>
      Seq(
        (u.lrs1_rtype === rtype, u.prs1),
        (u.lrs2_rtype === rtype, u.prs2),
        (u.frs3_en,              u.prs3)
      ).take(portsPerSlot).map { case (used, addr) => (v && used, addr) }
    }

    val mon = Module(new RegFileMonitor(name, hartId, numRegs, readEvents.length, writePorts.length,
      addrWidth, params))
    mon.suggestName(s"${name}_regfile_monitor")

    // The register file registers its read address, so the array is accessed one cycle after issue.
    for ((m, (valid, addr)) <- mon.io.reads zip readEvents) {
      m.valid := RegNext(valid, false.B)
      m.bits  := RegNext(addr)
    }
    for ((m, w) <- mon.io.writes zip writePorts) {
      m.valid := w.valid
      m.bits  := w.bits.addr
    }
    dontTouch(mon.io.dump)
    mon
  }
}

/**
 * Enable the register file usage monitor on all BOOM tiles.
 */
class WithRegFileMonitor(
  windowCycles: Int = 10000,
  counterBits: Option[Int] = None,
  dpiLog: Boolean = true,
  printDump: Boolean = false) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(core = tp.tileParams.core.copy(
      regfileMonitor = Some(RegFileMonitorParams(windowCycles, counterBits, dpiLog, printDump))
    )))
    case other => other
  }
})
