//******************************************************************************
// Execution Unit Occupancy Monitor
//------------------------------------------------------------------------------
//
// A passive observer that counts, per functional unit (FU) of each execution
// unit, over fixed windows of `windowCycles` cycles:
//  - issued: micro-ops issued to the FU (exe unit req valid with that FU's
//            fu_code bit)
//  - busy:   cycles the FU could not accept a new micro-op (its bit missing
//            from the exe unit's fu_types). Non-zero only for iterative or
//            queued FUs (div, fdiv, f2i); pipelined FUs accept every cycle.
// Occupancy of an FU in a window is max(issued, busy) / cycles.
//
// The monitor does not modify the execution units. It is instantiated by the
// parent (core for the integer/memory units, fp-pipeline for the FP units)
// only when BoomCoreParams.exeUnitMonitor is set (see WithExeUnitMonitor).
//
// Log: kind "exeunit", fields issued/busy, one row per FU, row names
// "<unit>.<fu>" (e.g. "alu0.div", "mem0.mem", "fp0.fdiv").

package boom.monitors

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Config
import freechips.rocketchip.subsystem.{TilesLocated, InSubsystem}

import boom.common._
import boom.exu.{ExecutionUnit, ExecutionUnits}
import boom.exu.FUConstants._

/**
 * @param tag log name ("exu_int" / "exu_fp")
 * @param hartId static tile id, used to name output files
 * @param rowNames one name per monitored FU
 */
class ExeUnitMonitor(
  tag: String,
  hartId: Int,
  rowNames: Seq[String],
  params: MonitorParams) extends Module
{
  private val R = rowNames.length
  val counterBits = log2Ceil(params.windowCycles + 1)

  val io = IO(new Bundle {
    val issued = Input(Vec(R, Bool()))
    val busy   = Input(Vec(R, Bool()))
    val dump   = Output(Valid(new MonitorRecord(R, 2, counterBits)))
  })

  val log = Module(new MonitorLogger(
    MonitorLogInfo("exeunit", tag, hartId, Seq("issued", "busy"), R, rowNames),
    counterBits, params.windowCycles, params.dpiLog, params.printDump))
  io.dump := log.io.dump

  val issuedCnt = RegInit(VecInit(Seq.fill(R)(0.U(counterBits.W))))
  val busyCnt   = RegInit(VecInit(Seq.fill(R)(0.U(counterBits.W))))

  for (r <- 0 until R) {
    val issuedNext = issuedCnt(r) + io.issued(r)
    val busyNext   = busyCnt(r) + io.busy(r)

    // Events in the last cycle of a window belong to that window.
    issuedCnt(r) := Mux(log.io.windowEnd, 0.U, issuedNext)
    busyCnt(r)   := Mux(log.io.windowEnd, 0.U, busyNext)

    log.io.snap(0)(r) := issuedNext
    log.io.snap(1)(r) := busyNext
    log.io.live(0)(r) := issuedCnt(r)
    log.io.live(1)(r) := busyCnt(r)
  }
}

object ExeUnitMonitor
{
  // (present in unit, short name, fu_code bit)
  private def fus(u: ExecutionUnit): Seq[(String, UInt)] = Seq(
    (u.hasAlu,     "alu",  FU_ALU),
    (u.hasJmpUnit, "jmp",  FU_JMP),
    (u.hasMem,     "mem",  FU_MEM),
    (u.hasMul,     "mul",  FU_MUL),
    (u.hasDiv,     "div",  FU_DIV),
    (u.hasCSR,     "csr",  FU_CSR),
    (u.hasIfpu,    "i2f",  FU_I2F),
    (u.hasFpu,     "fpu",  FU_FPU),
    (u.hasFdiv,    "fdiv", FU_FDV),
    (u.hasFpiu,    "f2i",  FU_F2I)
  ).collect { case (true, n, c) => (n, c) }

  /**
   * Build a monitor that observes a set of execution units from their parent.
   */
  def apply(tag: String, hartId: Int, units: ExecutionUnits, params: MonitorParams): ExeUnitMonitor =
  {
    val unitNames = {
      val seen = scala.collection.mutable.Map[String, Int]().withDefaultValue(0)
      units.map { u =>
        val kind = if (u.hasFpu) "fp" else if (u.hasMem && !u.hasAlu) "mem" else "alu"
        val n = s"$kind${seen(kind)}"
        seen(kind) += 1
        n
      }
    }

    // (row name, issued, busy) per FU
    val taps = units.map(u => u).zip(unitNames).flatMap { case (u, un) =>
      fus(u).map { case (fn, code) =>
        (s"$un.$fn",
         u.io.req.valid && (u.io.req.bits.uop.fu_code & code).orR,
         !(u.io.fu_types & code).orR)
      }
    }.toSeq

    val mon = Module(new ExeUnitMonitor(tag, hartId, taps.map(_._1), params))
    mon.suggestName(s"${tag}_monitor")
    for (((_, issued, busy), r) <- taps.zipWithIndex) {
      mon.io.issued(r) := issued
      mon.io.busy(r)   := busy
    }
    dontTouch(mon.io.dump)
    mon
  }
}

/**
 * Enable the execution unit occupancy monitor on all BOOM tiles.
 */
class WithExeUnitMonitor(
  windowCycles: Int = 10000,
  dpiLog: Boolean = true,
  printDump: Boolean = false) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(core = tp.tileParams.core.copy(
      exeUnitMonitor = Some(MonitorParams(windowCycles, dpiLog, printDump))
    )))
    case other => other
  }
})
