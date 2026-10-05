//******************************************************************************
// Register File Usage Monitor
//------------------------------------------------------------------------------
//
// A passive observer that counts, per physical register, how many times it is
// read and written. Counts are collected over fixed windows of `windowCycles`
// cycles; at the end of each window the counts are snapshotted, the live
// counters reset, and the snapshot is streamed out on `io.dump` one register
// per cycle as (window, row = preg, values = reads, writes).
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
// Windowing, snapshot/streaming and the output sinks (DPI-C binary log,
// printf) live in MonitorLogger. The log has fields reads/writes with one row
// per physical register.

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
  // Every port can hit the same register in a cycle.
  private val maxCount    = BigInt(params.windowCycles) * (numReadPorts max numWritePorts)
  val counterBits: Int    = params.counterBits.getOrElse(log2Ceil(maxCount + 1))

  require(maxCount < (BigInt(1) << counterBits),
    s"RegFileMonitor($name): counterBits ($counterBits) too small for windowCycles (${params.windowCycles})")

  val logInfo = MonitorLogInfo("regfile", name, hartId, Seq("reads", "writes"), numRegs)

  val io = IO(new Bundle {
    val reads  = Input(Vec(numReadPorts, Valid(UInt(addrWidth.W))))
    val writes = Input(Vec(numWritePorts, Valid(UInt(addrWidth.W))))
    val dump   = Output(Valid(new MonitorRecord(numRegs, 2, counterBits)))
  })

  val log = Module(new MonitorLogger(logInfo, counterBits, params.windowCycles,
    params.dpiLog, params.printDump))
  io.dump := log.io.dump

  // --------------------------------------------------------------
  // Live counters

  val rdCnt = RegInit(VecInit(Seq.fill(numRegs)(0.U(counterBits.W))))
  val wrCnt = RegInit(VecInit(Seq.fill(numRegs)(0.U(counterBits.W))))

  for (r <- 0 until numRegs) {
    val rdNext = rdCnt(r) + PopCount(io.reads.map(e => e.valid && e.bits === r.U))
    val wrNext = wrCnt(r) + PopCount(io.writes.map(e => e.valid && e.bits === r.U))

    // Events in the last cycle of a window belong to that window.
    rdCnt(r) := Mux(log.io.windowEnd, 0.U, rdNext)
    wrCnt(r) := Mux(log.io.windowEnd, 0.U, wrNext)

    log.io.snap(0)(r) := rdNext
    log.io.snap(1)(r) := wrNext
    log.io.live(0)(r) := rdCnt(r)
    log.io.live(1)(r) := wrCnt(r)
  }
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
