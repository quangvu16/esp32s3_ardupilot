package org.ardupilot.esp32gcs.desktop

import org.ardupilot.esp32gcs.nav.navSitlSnapshot
import java.awt.image.BufferedImage
import java.io.File
import javax.imageio.ImageIO

/** Runs the SITL navigation harness and saves the Indoor tab's map as PNGs during the patrol. */
fun main(args: Array<String>) {
    System.setProperty("java.awt.headless", "true")
    val out = File(args.getOrElse(0) { "/tmp/navsitl" })
    navSitlSnapshot = { nav, k ->
        val p = IndoorPanel(nav)
        p.setSize(900, 700)
        p.vp.follow = false
        val img = BufferedImage(900, 700, BufferedImage.TYPE_INT_ARGB)
        val g = img.createGraphics()
        p.paint(g); g.dispose()
        ImageIO.write(img, "png", File(out, "indoor_$k.png"))
        println("screenshot indoor_$k.png")
    }
    org.ardupilot.esp32gcs.nav.main(args)
}
