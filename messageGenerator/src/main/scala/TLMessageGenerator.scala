package messageGenerator

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{BundleFieldBase, BundleKeyBase}
import org.chipsalliance.cde.config.Parameters

case class TLMessageGeneratorParams(
  blockBytes: Int,
  lines: Int = 16,
  clientName: String = "MsgGen",
  sourceIdRange: IdRange = IdRange(0, 1),
  requestFields: Seq[BundleFieldBase] = Nil,
  responseKeys: Seq[BundleKeyBase] = Nil,
  supportsProbe: Option[TransferSizes] = None,
  minLatency: Int = 1
)

class TLMessageGenerator(params: TLMessageGeneratorParams)(implicit p: Parameters) extends LazyModule {
  private val blockBytes = params.blockBytes
  private val supportsProbe = params.supportsProbe.getOrElse(TransferSizes(blockBytes))
  require(params.sourceIdRange.size == 1, "TLMessageGenerator currently supports exactly one sourceId")

  val node = TLClientNode(Seq(
    TLMasterPortParameters.v2(
      masters = Seq(
        TLMasterParameters.v1(
          name = params.clientName,
          sourceId = params.sourceIdRange,
          supportsProbe = supportsProbe
        )
      ),
      channelBytes = TLChannelBeatBytes(blockBytes),
      minLatency = params.minLatency,
      requestFields = params.requestFields,
      echoFields = Nil,
      responseKeys = params.responseKeys
    )
  ))

  class TLMessageGeneratorImp(wrapper: TLMessageGenerator) extends LazyModuleImp(wrapper) {
    val out  = node.out.head._1
    val edge = node.out.head._2

    val io = IO(new Bundle {
      val in_addr       = Input(UInt(edge.bundle.addressBits.W))
      val in_isAcquire  = Input(Bool())
      val in_param      = Input(Bool())
    })

    val lines = params.lines
    val beatBytes = edge.bundle.dataBits / 8
    require(blockBytes % beatBytes == 0, s"blockBytes ($blockBytes) must be a multiple of beatBytes ($beatBytes)")
    val blockBits = blockBytes * 8
    val lineBeats = (blockBytes / beatBytes) max 1
    val beatBits  = beatBytes * 8
    val beatAddrShift = log2Ceil(beatBytes)
    val beatCountBits = scala.math.max(log2Ceil(lineBeats), 1)
    val offBits   = log2Ceil(blockBytes)
    val idxBits   = log2Ceil(lines)
    val tagBits   = edge.bundle.addressBits - idxBits - offBits
    val sourceId  = params.sourceIdRange.start.U(edge.bundle.sourceBits.W)

    private val releaseSourceC = Module(new MsgGenSourceC(edge, blockBytes, beatBytes))
    releaseSourceC.io.source := sourceId

    def addrIndex(a: UInt) = (a >> offBits)(idxBits-1,0)
    def addrTag(a: UInt)   = (a >> (offBits + idxBits))(tagBits-1,0)
    def lineBase(a: UInt)  = Cat(a(edge.bundle.addressBits-1, offBits), 0.U(offBits.W))

    val dir_valid = RegInit(VecInit(Seq.fill(lines)(false.B)))
    val dir_tag   = Reg(Vec(lines, UInt(tagBits.W)))
    val dir_state = RegInit(VecInit(Seq.fill(lines)(0.U(2.W))))
    val dir_data  = Reg(Vec(lines, UInt(blockBits.W)))

    val multiBeatLine = lineBeats > 1
    val probeActive       = RegInit(false.B)
    val probeBeatIdx      = RegInit(0.U(beatCountBits.W))
    val probeStoredData   = RegInit(0.U(blockBits.W))
    val probeStoredBase   = RegInit(0.U(edge.bundle.addressBits.W))

    val sIdle :: sWaitGrant :: sEnqueueRelease :: sWaitReleaseAck :: sGrantAck :: Nil = Enum(5)
    val state = RegInit(sIdle)

    val req_isAcquire  = Reg(Bool())
    val req_needT_or_toB = Reg(Bool())
    val req_addr       = Reg(UInt(edge.bundle.addressBits.W))
    val req_index      = addrIndex(req_addr)
    val req_tag        = addrTag(req_addr)

    private val releaseReqReg   = RegInit(0.U.asTypeOf(releaseSourceC.io.req.bits))
    val releaseReqValid = RegInit(false.B)

    val takeAcquire = (state === sIdle) && io.in_isAcquire
    val takeRelease = (state === sIdle) && !io.in_isAcquire
    val takeAddrIndex = addrIndex(io.in_addr)
    val takeAddrTag   = addrTag(io.in_addr)
    val takeLineHit   = dir_valid(takeAddrIndex) && dir_tag(takeAddrIndex) === takeAddrTag
    when (takeAcquire || takeRelease) {
      req_isAcquire      := io.in_isAcquire
      req_needT_or_toB   := io.in_param
      req_addr           := io.in_addr
      state := Mux(io.in_isAcquire, sWaitGrant, Mux(takeLineHit, sEnqueueRelease, sIdle))
      when (takeRelease && takeLineHit) {
        releaseReqReg.address := lineBase(io.in_addr)
        releaseReqReg.param   := Mux(io.in_param, TLPermissions.TtoB, TLPermissions.TtoN)
        releaseReqReg.data    := dir_data(takeAddrIndex)
        releaseReqValid       := true.B
      }
    }

    out.a.valid := false.B
    out.a.bits  := DontCare
    out.c.valid := false.B
    out.c.bits  := DontCare
    out.e.valid := false.B
    out.e.bits  := DontCare
    out.b.ready := !probeActive
    out.d.ready := true.B

    val probeHitIndex = addrIndex(out.b.bits.address)
    val probeHitTag   = addrTag(out.b.bits.address)
    val probeHit      = dir_valid(probeHitIndex) && dir_tag(probeHitIndex) === probeHitTag
    val probeData     = dir_data(probeHitIndex)
    val probeState    = dir_state(probeHitIndex)
    val probeBaseAddr = lineBase(out.b.bits.address)
    val sendingProbeAck = out.b.valid && out.b.ready && probeHit && (state =/= sGrantAck)

    val sourceCOut = releaseSourceC.io.tlc
    val cBitsWire = Wire(out.c.bits.cloneType)
    cBitsWire := sourceCOut.bits
    val cValidWire = WireInit(sourceCOut.valid)
    val sourceCReady = WireInit(out.c.ready && !sendingProbeAck && !probeActive)

    def mkProbeAckBundle(addr: UInt, data: UInt) = {
      val bundle = WireDefault(0.U.asTypeOf(out.c.bits))
      bundle.opcode := TLMessages.ProbeAckData
      bundle.param  := 0.U
      bundle.source := sourceId
      bundle.address:= addr
      bundle.size   := log2Ceil(blockBytes).U
      bundle.data   := data
      bundle.corrupt:= false.B
      bundle
    }

    when (sendingProbeAck) {
      val probeVec = probeData.asTypeOf(Vec(lineBeats, UInt(beatBits.W)))
      val firstBeat = probeVec.head
      cBitsWire := mkProbeAckBundle(probeBaseAddr, firstBeat)
      cValidWire := true.B
      sourceCReady := false.B
      when (out.c.fire) {
        when (probeState === 2.U) { dir_state(probeHitIndex) := 1.U }
        if (multiBeatLine) {
          probeActive := true.B
          probeBeatIdx := 1.U
          probeStoredData := probeData
          probeStoredBase := probeBaseAddr
        } else {
          probeActive := false.B
          probeBeatIdx := 0.U
        }
      }
    }

    if (multiBeatLine) {
      val storedVec = probeStoredData.asTypeOf(Vec(lineBeats, UInt(beatBits.W)))
      when (probeActive && !sendingProbeAck) {
        val beatData = storedVec(probeBeatIdx)
        val beatAddr = probeStoredBase + (probeBeatIdx << beatAddrShift).asUInt
        cBitsWire := mkProbeAckBundle(beatAddr, beatData)
        cValidWire := true.B
        sourceCReady := false.B
        when (out.c.fire) {
          val lastBeat = probeBeatIdx === (lineBeats - 1).U
          when (lastBeat) {
            probeActive := false.B
            probeBeatIdx := 0.U
          }.otherwise {
            probeBeatIdx := probeBeatIdx + 1.U
          }
        }
      }
    } else {
      when (!sendingProbeAck) {
        probeActive := false.B
        probeBeatIdx := 0.U
      }
    }

    sourceCOut.ready := sourceCReady
    out.c.valid := cValidWire
    out.c.bits  := cBitsWire

    val issuingAcquire = (state === sWaitGrant) && req_isAcquire && !sendingProbeAck
    when (issuingAcquire) {
      val growPerm = Mux(req_needT_or_toB, TLPermissions.NtoT, TLPermissions.NtoB)
      val (_, acquire) = edge.AcquireBlock(sourceId, lineBase(req_addr), log2Ceil(blockBytes).U, growPerm)
      out.a.valid := true.B
      out.a.bits  := acquire
    }

    val relIdx = req_index

    releaseSourceC.io.req.valid := (state === sEnqueueRelease) && releaseReqValid
    releaseSourceC.io.req.bits  := releaseReqReg

    when (releaseSourceC.io.req.fire) {
      releaseReqValid := false.B
      dir_valid(relIdx) := false.B
      dir_state(relIdx) := 0.U
      state := sWaitReleaseAck
    }

    val d_isReleaseAck = out.d.valid && out.d.bits.opcode === TLMessages.ReleaseAck
    val grantAccumulating = RegInit(false.B)
    val grantBeatIdx      = RegInit(0.U(beatCountBits.W))
    val grantBeats        = Reg(Vec(lineBeats, UInt(beatBits.W)))

    when (out.d.fire && out.d.bits.opcode === TLMessages.GrantData) {
      val idx = addrIndex(req_addr)
      val tag = addrTag(req_addr)
      val updatedBeats = Wire(Vec(lineBeats, UInt(beatBits.W)))
      for (i <- 0 until lineBeats) {
        val prev = grantBeats(i)
        updatedBeats(i) := Mux(i.U === grantBeatIdx, out.d.bits.data, prev)
        grantBeats(i) := updatedBeats(i)
      }
      when (!grantAccumulating) {
        grantAccumulating := true.B
        grantBeatIdx      := 0.U
      }

      val fullLine = if (lineBeats == 1) {
        updatedBeats.head
      } else {
        Cat(updatedBeats.reverse)
      }

      when (edge.last(out.d)) {
        dir_valid(idx) := true.B
        dir_tag(idx)   := tag
        dir_state(idx) := Mux(req_needT_or_toB, 2.U, 1.U)
        dir_data(idx)  := fullLine
        grantAccumulating := false.B
        grantBeatIdx      := 0.U
        state := sGrantAck
      }.otherwise {
        grantBeatIdx := grantBeatIdx + 1.U
      }
    }

    when (out.d.fire && out.d.bits.opcode === TLMessages.Grant) {
      val idx = addrIndex(req_addr)
      val tag = addrTag(req_addr)
      dir_valid(idx) := true.B
      dir_tag(idx)   := tag
      dir_state(idx) := Mux(req_needT_or_toB, 2.U, 1.U)
      val synth = Fill(beatBytes, tag(7,0))
      val expanded = Fill(blockBytes/beatBytes, synth)
      dir_data(idx)  := expanded
      state := sGrantAck
    }

    when (d_isReleaseAck && out.d.fire) {
      state := sIdle
    }

    when (state === sGrantAck) {
      out.e.valid := true.B
      out.e.bits.sink := out.d.bits.sink
      when (out.e.fire) { state := sIdle }
    }

    when (!out.a.valid) { out.a.bits := 0.U.asTypeOf(out.a.bits) }
    when (!out.c.valid) { out.c.bits := 0.U.asTypeOf(out.c.bits) }
    when (!out.e.valid) { out.e.bits := 0.U.asTypeOf(out.e.bits) }

    val io_in_addr      = io.in_addr
    val io_in_isAcquire = io.in_isAcquire
    val io_in_param     = io.in_param
  }

  lazy val module = new TLMessageGeneratorImp(this)
}
