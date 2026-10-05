//******************************************************************************
// Free List Monitor
//------------------------------------------------------------------------------
//
// A passive observer of a rename stage's physical register free list. Over
// fixed windows of `windowCycles` cycles it records:
//  - free_sum:     sum over cycles of the number of free physical registers
//                  (average = free_sum / window cycles)
//  - free_min:     fewest free registers seen in any cycle
//  - free_max:     most free registers seen in any cycle
//  - stall_cycles: cycles in which rename stalled because the free list could
//                  not supply a register (any valid lane with ren_stalls set)
//
// The free count is the popcount of the rename stage's debug free list, which
// includes the registers currently offered for allocation.
//
// The monitor does not modify the rename stage. It is instantiated by the core
// (one per int / fp rename stage) only when BoomCoreParams.freeListMonitor is
// set (see WithFreeListMonitor).
//
// Log: kind "freelist", the four fields above, one row, extra meta "num_regs".

package boom.monitors

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Config
import freechips.rocketchip.subsystem.{TilesLocated, InSubsystem}

import boom.common._
import boom.exu.RenameStage

/**
 * @param tag log name ("freelist_int" / "freelist_fp")
 * @param hartId static tile id, used to name output files
 * @param numRegs number of physical registers in the free list
 */
class FreeListMonitor(
  tag: String,
  hartId: Int,
  numRegs: Int,
  params: MonitorParams) extends Module
{
  val counterBits = log2Ceil(BigInt(numRegs) * params.windowCycles + 1)
  private val countBits = log2Ceil(numRegs + 1)

  val io = IO(new Bundle {
    val freelist = Input(UInt(numRegs.W))
    val stall    = Input(Bool())
    val dump     = Output(Valid(new MonitorRecord(1, 4, counterBits)))
  })

  val log = Module(new MonitorLogger(
    MonitorLogInfo("freelist", tag, hartId, Seq("free_sum", "free_min", "free_max", "stall_cycles"), 1,
      extra = Seq("num_regs" -> BigInt(numRegs))),
    counterBits, params.windowCycles, params.dpiLog, params.printDump))
  io.dump := log.io.dump

  val free = PopCount(io.freelist)

  val sum   = RegInit(0.U(counterBits.W))
  val min   = RegInit(numRegs.U(countBits.W))
  val max   = RegInit(0.U(countBits.W))
  val stall = RegInit(0.U(counterBits.W))

  val sumNext   = sum + free
  val minNext   = Mux(free < min, free, min)
  val maxNext   = Mux(free > max, free, max)
  val stallNext = stall + io.stall

  // This cycle belongs to the window that ends now.
  val end = log.io.windowEnd
  sum   := Mux(end, 0.U, sumNext)
  min   := Mux(end, numRegs.U, minNext)
  max   := Mux(end, 0.U, maxNext)
  stall := Mux(end, 0.U, stallNext)

  log.io.snap(0)(0) := sumNext
  log.io.snap(1)(0) := minNext
  log.io.snap(2)(0) := maxNext
  log.io.snap(3)(0) := stallNext
  log.io.live(0)(0) := sum
  log.io.live(1)(0) := min
  log.io.live(2)(0) := max
  log.io.live(3)(0) := stall
}

object FreeListMonitor
{
  /**
   * Build a monitor that observes a rename stage's free list from its parent.
   */
  def apply(tag: String, hartId: Int, numRegs: Int, rename: RenameStage,
            params: MonitorParams): FreeListMonitor =
  {
    val mon = Module(new FreeListMonitor(tag, hartId, numRegs, params))
    mon.suggestName(s"${tag}_monitor")
    mon.io.freelist := rename.io.debug.freelist
    mon.io.stall    := (rename.io.ren2_mask zip rename.io.ren_stalls).map { case (v, s) => v && s }.reduce(_||_)
    dontTouch(mon.io.dump)
    mon
  }
}

/**
 * Enable the free list monitor on all BOOM tiles.
 */
class WithFreeListMonitor(
  windowCycles: Int = 10000,
  dpiLog: Boolean = true,
  printDump: Boolean = false) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(core = tp.tileParams.core.copy(
      freeListMonitor = Some(MonitorParams(windowCycles, dpiLog, printDump))
    )))
    case other => other
  }
})

/**
 * Enable every monitor with one shared window size, so their windows align.
 */
class WithAllMonitors(windowCycles: Int = 10000, dpiLog: Boolean = true) extends Config(
  new WithRegFileMonitor(windowCycles = windowCycles, dpiLog = dpiLog) ++
  new WithExeUnitMonitor(windowCycles = windowCycles, dpiLog = dpiLog) ++
  new WithFreeListMonitor(windowCycles = windowCycles, dpiLog = dpiLog))
