# Message Generator Standalone

该目录包含一个从 CoupledL2 验证环境中抽离出来的 TileLink 消息发生器，并以独立 Mill 工程的形式组织。工程默认依赖 Chisel 6.7.0 (Scala 2.13.12)，同时需要 Rocket Chip 的 TileLink/diplomacy 等源码作为编译依赖。

## 目录结构

```
message-generator-standalone/
├── build.sc          # Mill 构建脚本
├── common.sc         # Chisel 相关公用配置
├── README.md         # 项目说明
└── src/
    └── main/
        └── scala/
            └── messageGenerator/
                ├── MsgGenSourceC.scala
                └── TLMessageGenerator.scala
```
