package org.ardupilot.esp32gcs.desktop

import org.ardupilot.esp32gcs.Platform
import java.util.prefs.Preferences
import javax.swing.JFrame
import javax.swing.SwingUtilities
import javax.swing.UIManager

/** ESP32 Rover GCS for Windows (and any desktop with Java 17). */
fun main() {
    // Vehicle events arrive on the Swing thread, like Android's main thread.
    Platform.post = { SwingUtilities.invokeLater(it) }
    runCatching { UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName()) }
    SwingUtilities.invokeLater {
        val prefs = Preferences.userRoot().node("esp32-rover-gcs")
        val gcs = GcsPanel()
        JFrame("ESP32 Rover GCS").apply {
            defaultCloseOperation = JFrame.EXIT_ON_CLOSE
            contentPane = gcs
            setSize(prefs.getInt("w", 1280), prefs.getInt("h", 820))
            setLocationRelativeTo(null)
            addWindowListener(object : java.awt.event.WindowAdapter() {
                override fun windowClosing(e: java.awt.event.WindowEvent?) {
                    gcs.shutdown(); prefs.putInt("w", width); prefs.putInt("h", height)
                }
            })
            isVisible = true
        }
    }
}
