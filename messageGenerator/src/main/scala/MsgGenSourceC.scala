package messageGenerator

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._


class MsgGenReleaseReq(edge: TLEdgeOut, blockBytes: Int) extends Bundle {
  val address = UInt(edge.bundle.addressBits.W)
  val param   = UInt(3.W)
  val data    = UInt((blockBytes * 8).W)
}

class MsgGenSourceC(edge: TLEdgeOut, blockBytes: Int, beatBytes: Int) extends Module {
  private val lineBeats     = (blockBytes / beatBytes) max 1
  private val beatBits      = beatBytes * 8
  private val beatAddrShift = log2Ceil(beatBytes)
  private val beatIdxBits   = (log2Ceil(lineBeats) max 1)

  val io = IO(new Bundle {
    val req    = Flipped(Decoupled(new MsgGenReleaseReq(edge, blockBytes)))
    val tlc    = Decoupled(new TLBundleC(edge.bundle))
    val source = Input(UInt(edge.bundle.sourceBits.W))
  })

  val active = RegInit(false.B)
  val beatIdx = RegInit(0.U(beatIdxBits.W))
  val reqReg = RegInit(0.U.asTypeOf(new MsgGenReleaseReq(edge, blockBytes)))

  io.req.ready := !active

  when(io.req.fire) {
    reqReg := io.req.bits
    active := true.B
    beatIdx := 0.U
  }

  val cBundle = WireDefault(0.U.asTypeOf(new TLBundleC(edge.bundle)))
  val dataVec = reqReg.data.asTypeOf(Vec(lineBeats, UInt(beatBits.W)))
  val beatData = dataVec(beatIdx)
  val addrWithOffset = reqReg.address + (beatIdx << beatAddrShift).asUInt

  cBundle.opcode := TLMessages.ReleaseData
  cBundle.param  := reqReg.param
  cBundle.size   := log2Ceil(blockBytes).U
  cBundle.source := io.source
  cBundle.address:= addrWithOffset
  cBundle.data   := beatData
  cBundle.corrupt:= false.B

  io.tlc.valid := active
  io.tlc.bits  := cBundle

  val lastBeat = beatIdx === (lineBeats - 1).U

  when(io.tlc.fire) {
    when(lastBeat) {
      active := false.B
    }.otherwise {
      beatIdx := beatIdx + 1.U
    }
  }
}
