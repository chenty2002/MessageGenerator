import mill._
import scalalib._

trait HasChisel6 extends ScalaModule {
  def chiselVersion: String = "6.7.0"

  override def scalaVersion = "2.13.12"

  override def scalacOptions = super.scalacOptions() ++
    Agg("-language:reflectiveCalls", "-Ymacro-annotations", "-Ytasty-reader")

  override def ivyDeps = super.ivyDeps() ++ Agg(
    ivy"org.chipsalliance::chisel:${chiselVersion}"
  )

  override def scalacPluginIvyDeps = super.scalacPluginIvyDeps() ++ Agg(
    ivy"org.chipsalliance:::chisel-plugin:${chiselVersion}"
  )
}

trait RequiresRocketChip extends ScalaModule {
  def rocketChipModule: ScalaModule

  override def moduleDeps = super.moduleDeps ++ Seq(rocketChipModule)
}
