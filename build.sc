import mill._
import scalalib._
import scalafmt._
import $file.common

object fmt extends ScalafmtModule

object deps extends Module {
  private val depsRoot = os.pwd / "deps"
  private val rocketChipRoot = depsRoot / "rocket-chip"

  object macros extends ScalaModule with common.HasChisel6 {
    override def millSourcePath = rocketChipRoot / "macros"

    override def ivyDeps = T {
      super.ivyDeps() ++ Agg(ivy"org.scala-lang:scala-reflect:${scalaVersion()}")
    }
  }

  object cde extends ScalaModule with common.HasChisel6 {
    override def millSourcePath = rocketChipRoot / "dependencies" / "cde" / "cde"
  }

  object hardfloat extends ScalaModule with common.HasChisel6 {
    override def millSourcePath = rocketChipRoot / "dependencies" / "hardfloat" / "hardfloat"

    override def moduleDeps = super.moduleDeps ++ Seq(macros)
  }

  object diplomacy extends ScalaModule with common.HasChisel6 {
    override def millSourcePath = rocketChipRoot / "dependencies" / "diplomacy" / "diplomacy"

    override def moduleDeps = super.moduleDeps ++ Seq(cde)

    override def ivyDeps = T {
      super.ivyDeps() ++ Agg(ivy"com.lihaoyi::sourcecode:0.3.1")
    }
  }

  object rocketchip extends ScalaModule with common.HasChisel6 {
    override def millSourcePath = rocketChipRoot

    override def moduleDeps = super.moduleDeps ++ Seq(macros, hardfloat, cde, diplomacy)

    override def ivyDeps = T {
      super.ivyDeps() ++ Agg(
        ivy"com.lihaoyi::mainargs:0.5.0",
        ivy"org.json4s::json4s-jackson:4.0.5"
      )
    }
  }
}

object messageGenerator extends ScalaModule with common.HasChisel6 with common.RequiresRocketChip {
  override def rocketChipModule = deps.rocketchip

  override def sources = T.sources(
    millSourcePath / "src" / "main" / "scala"
  )
}
