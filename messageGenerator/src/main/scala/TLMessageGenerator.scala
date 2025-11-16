package messageGenerator

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{BundleFieldBase, BundleKeyBase}
import org.chipsalliance.cde.config.Parameters

case class MessageGeneratorParam(
  name: String = "MsgGen",
  sets: Int = 16,
  ways: Int = 1,
  blockBytes: Int = 64,
  channelBytes: TLChannelBeatBytes = TLChannelBeatBytes(32),
  
  // TileLink client parameters
  sourceIdRange: IdRange = IdRange(0, 16),
  reqField: Seq[BundleFieldBase] = Nil,
  respKey: Seq[BundleKeyBase] = Nil,
  supportsProbe: Option[TransferSizes] = None,
  minLatency: Int = 1
) {
  // Derived values for compatibility
  def lines: Int = sets * ways
  def clientName: String = name
  def requestFields: Seq[BundleFieldBase] = reqField
  def responseKeys: Seq[BundleKeyBase] = respKey
}

class TLMessageGenerator(params: MessageGeneratorParam)(implicit p: Parameters) extends LazyModule {
  private val blockBytes = params.blockBytes
  private val supportsProbe = params.supportsProbe.getOrElse(TransferSizes(blockBytes))
  private val maxSources = params.sourceIdRange.size
  require(maxSources > 0 && maxSources <= 16, "TLMessageGenerator expects 1 to 16 sourceIds")

  val node = TLClientNode(Seq(
    TLMasterPortParameters.v2(
      masters = Seq(
        TLMasterParameters.v1(
          name = params.name,
          sourceId = params.sourceIdRange,
          supportsProbe = supportsProbe
        )
      ),
      channelBytes = params.channelBytes,
      minLatency = params.minLatency,
      requestFields = params.reqField,
      echoFields = Nil,
      responseKeys = params.respKey
    )
  ))

  class TLMessageGeneratorImp(wrapper: TLMessageGenerator) extends LazyModuleImp(wrapper) {
    val out  = node.out.head._1
    val edge = node.out.head._2

    val io = IO(new Bundle {
      val in_addr       = Input(UInt(edge.bundle.addressBits.W))
      val in_isAcquire  = Input(Bool())  // true=Acquire, false=Release
      val in_param      = Input(Bool())  // Acquire:T/B权限; Release:toB/toN
      val in_data       = Input(UInt(blockBits.W))  // Release时使用的数据
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
    val numSources = maxSources
    val sourceIdVec = VecInit((0 until numSources).map(i => (params.sourceIdRange.start + i).U(edge.bundle.sourceBits.W)))
    val defaultSourceId = params.sourceIdRange.start.U(edge.bundle.sourceBits.W)

    private val releaseSourceC = Module(new MsgGenSourceC(edge, blockBytes, beatBytes, entries = 8))
    releaseSourceC.io.source := defaultSourceId

    def addrIndex(a: UInt) = (a >> offBits)(idxBits-1,0)
    def addrTag(a: UInt)   = (a >> (offBits + idxBits))(tagBits-1,0)
    def lineBase(a: UInt)  = Cat(a(edge.bundle.addressBits-1, offBits), 0.U(offBits.W))

    // 目录状态枚举
    val stateN :: stateB :: stateT :: Nil = Enum(3)
    
    val dir_valid = RegInit(VecInit(Seq.fill(lines)(false.B)))
    val dir_tag   = Reg(Vec(lines, UInt(tagBits.W)))
    val dir_state = RegInit(VecInit(Seq.fill(lines)(stateN)))  // N=None, B=Branch, T=Trunk
    val dir_data  = Reg(Vec(lines, UInt(blockBits.W)))

    // Acquire请求追踪
    val entryIdle :: entrySendAcquire :: entryWaitGrant :: entryGrantAck :: Nil = Enum(4)
    val entryActive = RegInit(VecInit(Seq.fill(numSources)(false.B)))
    val entryState = RegInit(VecInit(Seq.fill(numSources)(entryIdle)))
    val entryNeedT = Reg(Vec(numSources, Bool()))
    val entryAddr = Reg(Vec(numSources, UInt(edge.bundle.addressBits.W)))
    val entryGrantAccumulating = RegInit(VecInit(Seq.fill(numSources)(false.B)))  // 正在接收Grant beats
    val entryGrantBeatIdx = RegInit(VecInit(Seq.fill(numSources)(0.U(beatCountBits.W))))
    
    val entryGrantBeats = Reg(Vec(numSources, Vec(lineBeats, UInt(beatBits.W))))  // 缓存Grant数据
    val entryGrantSink = if (edge.bundle.sinkBits > 0) Some(RegInit(VecInit(Seq.fill(numSources)(0.U(edge.bundle.sinkBits.W))))) else None

    
    private val releaseReqReg   = Reg(new MsgGenSourceCReq(edge, blockBytes))
    val releaseReqValid = RegInit(false.B)

    // Release状态机
    val releaseIdle :: releaseEnqueue :: releaseWaitAck :: Nil = Enum(3)
    val releaseState = RegInit(releaseIdle)

    // Acquire分配空闲槽
    val takeAcquire = io.in_isAcquire
    val takeRelease = !io.in_isAcquire
    val takeAddrIndex = addrIndex(io.in_addr)
    val takeAddrTag   = addrTag(io.in_addr)
    val takeLineHit   = dir_valid(takeAddrIndex) && dir_tag(takeAddrIndex) === takeAddrTag
    val freeVec = entryActive.map(!_)
    val freeMask = VecInit(freeVec).asUInt
    val allocateOH = PriorityEncoderOH(freeMask)
    val canAllocate = freeMask.orR
    val allocateAcquire = takeAcquire && canAllocate
    val allocIdx = OHToUInt(allocateOH)

    when (allocateAcquire) {
      entryActive(allocIdx) := true.B
      entryState(allocIdx) := entrySendAcquire
      entryNeedT(allocIdx) := io.in_param
      entryAddr(allocIdx) := io.in_addr
      entryGrantAccumulating(allocIdx) := false.B
      entryGrantBeatIdx(allocIdx) := 0.U
    }

    // Release请求命中则加入队列
    when (takeRelease && takeLineHit && releaseState === releaseIdle) {
      releaseReqReg.opcode  := TLMessages.ReleaseData
      releaseReqReg.param   := Mux(io.in_param, TLPermissions.TtoB, TLPermissions.TtoN)
      releaseReqReg.address := lineBase(io.in_addr)
      releaseReqReg.data    := io.in_data
      releaseReqValid       := true.B
      releaseState := releaseEnqueue
    }

    // Assume: Release requests must hit a valid cache line
    assume(io.in_isAcquire || takeLineHit)

    // A通道发送Acquire
    val sendAcquireVec = VecInit((0 until numSources).map(i => entryActive(i) && entryState(i) === entrySendAcquire))
    val sendAcquireMask = sendAcquireVec.asUInt
    val sendAcquireSel = PriorityEncoderOH(sendAcquireMask)
    val hasAcquireToSend = sendAcquireMask.orR
    val acquireIndex = OHToUInt(sendAcquireSel)

    // B channel ready logic: no backpressure from ProbeAck queue
    out.b.ready := true.B
    out.d.ready := true.B

    // B通道接收Probe并发送ProbeAck
    val probeHitIndex = addrIndex(out.b.bits.address)
    val probeHitTag   = addrTag(out.b.bits.address)
    val probeHit      = dir_valid(probeHitIndex) && dir_tag(probeHitIndex) === probeHitTag
    val probeData     = dir_data(probeHitIndex)
    val probeState    = dir_state(probeHitIndex)
    val probeBaseAddr = lineBase(out.b.bits.address)
    val probeParam    = out.b.bits.param

    // Calculate ProbeAck opcode: ProbeAck or ProbeAckData
    val needProbeAckData = probeHit && probeState === stateT
    val probeAckOpcode = Mux(needProbeAckData, TLMessages.ProbeAckData, TLMessages.ProbeAck)

    // Calculate ProbeAck param based on current state and probe param
    val probeAckParam = Mux(!probeHit, TLPermissions.NtoN,
      MuxLookup(Cat(probeParam, probeState), TLPermissions.BtoB)(Seq(
        Cat(TLPermissions.toN, stateB) -> TLPermissions.BtoN,  // B→N
        Cat(TLPermissions.toN, stateT) -> TLPermissions.TtoN,  // T→N
        Cat(TLPermissions.toB, stateT) -> TLPermissions.TtoB,  // T→B
        Cat(TLPermissions.toB, stateB) -> TLPermissions.BtoB   // B→B (no change)
      ))
    )

    // Enqueue ProbeAck request to SourceC
    val probeAckReq = Wire(new MsgGenSourceCReq(edge, blockBytes))
    probeAckReq.opcode  := probeAckOpcode
    probeAckReq.param   := probeAckParam
    probeAckReq.address := probeBaseAddr
    probeAckReq.data    := Mux(needProbeAckData, probeData, 0.U)

    // ProbeAck has higher priority than Release, directly connect to SourceC
    val probeAckValid = out.b.valid && out.b.ready
    
    when (out.b.fire) {
      // Update directory state based on probe param
      when (probeHit) {
        when (probeParam === TLPermissions.toN) {
          dir_valid(probeHitIndex) := false.B
          dir_state(probeHitIndex) := stateN
        }.elsewhen (probeParam === TLPermissions.toB && probeState === stateT) {
          dir_state(probeHitIndex) := stateB
        }
        // toB on stateB or toT on stateT: no state change
      }
    }

    // A channel: Acquire requests
    // Similar to CoupledL2 AcquireUnit: straightforward valid/ready assignment
    val allowAcquire = hasAcquireToSend && !probeAckValid
    val aBitsWire = Wire(out.a.bits.cloneType)
    aBitsWire := 0.U.asTypeOf(out.a.bits)
    when (hasAcquireToSend) {
      val growPerm = Mux(entryNeedT(acquireIndex), TLPermissions.NtoT, TLPermissions.NtoB)
      val (_, acquireMsg) = edge.AcquireBlock(sourceIdVec(acquireIndex), lineBase(entryAddr(acquireIndex)), log2Ceil(blockBytes).U, growPerm)
      aBitsWire := acquireMsg
    }
    out.a.valid := allowAcquire
    out.a.bits  := aBitsWire

    when (out.a.fire) {
      entryState(acquireIndex) := entryWaitGrant
    }

    // C channel: ProbeAck (priority) vs Release
    // Similar to CoupledL2 SourceC: handle multi-source output with proper arbitration
    val sourceCReqValid = Mux(probeAckValid, true.B, releaseState === releaseEnqueue && releaseReqValid)
    val sourceCReqBits = Mux(probeAckValid, probeAckReq, releaseReqReg)

    releaseSourceC.io.req.valid := sourceCReqValid
    releaseSourceC.io.req.bits  := sourceCReqBits
    
    out.c <> releaseSourceC.io.tlc

    // Release发送队列管理
    val releaseIndex = addrIndex(releaseReqReg.address)
    
    when (releaseSourceC.io.req.fire && !probeAckValid) {
      releaseReqValid := false.B
      dir_valid(releaseIndex) := false.B
      dir_state(releaseIndex) := stateN
      releaseState := releaseWaitAck
    }

    when (out.d.fire && out.d.bits.opcode === TLMessages.ReleaseAck) {
      releaseState := releaseIdle
      releaseReqValid := false.B
    }

    // D channel: Grant/GrantData handling
    // Similar to CoupledL2 RefillUnit: match source ID and accumulate beats
    val dSourceMatch = VecInit((0 until numSources).map(i => sourceIdVec(i) === out.d.bits.source))
    val dMatchedIdx = OHToUInt(dSourceMatch.asUInt)
    val dMatched = dSourceMatch.asUInt.orR

    when (out.d.fire && dMatched) {
      val matchedEntry = dMatchedIdx
      val lineIdx = addrIndex(entryAddr(matchedEntry))
      val lineTag = addrTag(entryAddr(matchedEntry))
      
      when (out.d.bits.opcode === TLMessages.GrantData) {
        entryGrantBeats(matchedEntry)(entryGrantBeatIdx(matchedEntry)) := out.d.bits.data
        
        when (!entryGrantAccumulating(matchedEntry)) {
          entryGrantAccumulating(matchedEntry) := true.B
          entryGrantBeatIdx(matchedEntry) := 0.U
        }

        // 多beat累积：最后一拍时写入目录
        when (edge.last(out.d)) {
          // Build fullLine only when complete
          val fullLine = if (lineBeats == 1) {
            entryGrantBeats(matchedEntry).head
          } else {
            Cat(entryGrantBeats(matchedEntry).reverse)
          }
          // 根据Grant的param更新entryNeedT: toT(0)=true, toB(1)=false
          val grantedT = out.d.bits.param === TLPermissions.toT
          entryNeedT(matchedEntry) := grantedT
          dir_valid(lineIdx) := true.B
          dir_tag(lineIdx)   := lineTag
          dir_state(lineIdx) := Mux(grantedT, stateT, stateB)
          dir_data(lineIdx)  := fullLine
          entryGrantAccumulating(matchedEntry) := false.B
          entryGrantBeatIdx(matchedEntry) := 0.U
          entryState(matchedEntry) := entryGrantAck
          if (edge.bundle.sinkBits > 0) { entryGrantSink.get(matchedEntry) := out.d.bits.sink }
        }.otherwise {
          entryGrantBeatIdx(matchedEntry) := entryGrantBeatIdx(matchedEntry) + 1.U
        }
      }.elsewhen (out.d.bits.opcode === TLMessages.Grant) {
        // 根据Grant的param更新entryNeedT: toT(0)=true, toB(1)=false
        val grantedT = out.d.bits.param === TLPermissions.toT
        entryNeedT(matchedEntry) := grantedT
        // 无数据Grant：目录写入0
        dir_valid(lineIdx) := true.B
        dir_tag(lineIdx)   := lineTag
        dir_state(lineIdx) := Mux(grantedT, stateT, stateB)
        dir_data(lineIdx)  := 0.U
        entryState(matchedEntry) := entryGrantAck
        entryGrantAccumulating(matchedEntry) := false.B
        entryGrantBeatIdx(matchedEntry) := 0.U
        if (edge.bundle.sinkBits > 0) { entryGrantSink.get(matchedEntry) := out.d.bits.sink }
      }
    }

    // E channel: GrantAck
    // Similar to CoupledL2 RefillUnit: straightforward enqueue and dequeue pattern
    val grantAckVec = VecInit((0 until numSources).map(i => entryActive(i) && entryState(i) === entryGrantAck))
    val grantAckMask = grantAckVec.asUInt
    val grantAckSel = PriorityEncoderOH(grantAckMask)
    val hasGrantAck = grantAckMask.orR
    val grantAckIdx = OHToUInt(grantAckSel)
    
    val eBitsWire = Wire(out.e.bits.cloneType)
    eBitsWire := 0.U.asTypeOf(out.e.bits)
    if (edge.bundle.sinkBits > 0) {
      when (hasGrantAck) { eBitsWire.sink := entryGrantSink.get(grantAckIdx) }
    }
    out.e.valid := hasGrantAck
    out.e.bits := eBitsWire

    when (out.e.fire) {
      entryActive(grantAckIdx) := false.B
      entryState(grantAckIdx) := entryIdle
    }

    when (!out.a.valid) { out.a.bits := 0.U.asTypeOf(out.a.bits) }

    val io_in_addr      = io.in_addr
    val io_in_isAcquire = io.in_isAcquire
    val io_in_param     = io.in_param
    val io_in_data      = io.in_data
  }

  lazy val module = new TLMessageGeneratorImp(this)
}
