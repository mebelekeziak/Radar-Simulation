using System;
using System.IO;

namespace RealRadarSim.Logging
{
    public static class DebugLogger
    {
        private static readonly string logFilePath = "debug.txt";

        /// <summary>
        /// Writes a line to the debug log only if it is about a missed track.
        /// </summary>
        public static void WriteLine(string message)
        {
            // Only output messages that are flagged as missed-track events.
            if (!message.StartsWith("[Missed Track]"))
                return;

            try
            {
                // Append the message with a timestamp.
                string timestampedMessage = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} - {message}";
                File.AppendAllText(logFilePath, timestampedMessage + Environment.NewLine);
            }
            catch (Exception ex)
            {
                // If logging fails, fallback to Console.
                Console.WriteLine("Error writing to log file: " + ex.Message);
            }
        }
    }
}
