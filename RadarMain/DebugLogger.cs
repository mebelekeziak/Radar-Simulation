using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace RealRadarSim.Logging
{
    /// <summary>
    /// An improved asynchronous debug logger that groups repeated messages,
    /// rotates the log file when it grows too large, and keeps a limited in‑memory log.
    /// </summary>
    public static class DebugLogger
    {
        // --- Configuration Settings ---
        private static readonly string logFilePath = "debug.txt";
        private const int MaxMessagesInMemory = 100;          // In‑memory buffer size.
        private const long MaxLogFileSizeBytes = 5 * 1024 * 1024; // 5 MB log file rotation.
        private static readonly TimeSpan GroupInterval = TimeSpan.FromMilliseconds(500);

        // --- In‑memory log storage ---
        private static readonly List<string> debugMessages = new List<string>();
        private static readonly object memoryLock = new object();

        // --- Asynchronous logging ---
        private static readonly ConcurrentQueue<string> logQueue = new ConcurrentQueue<string>();
        private static readonly CancellationTokenSource cts = new CancellationTokenSource();
        private static readonly Task logTask;

        // --- Grouping of repeated messages ---
        private static readonly object groupingLock = new object();
        private static readonly Dictionary<string, LogEntry> lastLogEntries = new Dictionary<string, LogEntry>();

        // Helper class for grouping repeated messages per category.
        private class LogEntry
        {
            public string Message;
            public int Count;
            public DateTime LastLogged;
        }

        // --- Static constructor starts the background logging task ---
        static DebugLogger()
        {
            logTask = Task.Factory.StartNew(ProcessQueue, TaskCreationOptions.LongRunning);
        }

        /// <summary>
        /// Gets a snapshot of the most recent debug messages (up to MaxMessagesInMemory).
        /// </summary>
        public static IReadOnlyList<string> DebugMessages
        {
            get
            {
                lock (memoryLock)
                {
                    return debugMessages.ToList();
                }
            }
        }

        /// <summary>
        /// Stops the background logging task. Call this on application shutdown to flush the log.
        /// </summary>
        public static void Shutdown()
        {
            cts.Cancel();
            logTask.Wait();
        }

        /// <summary>
        /// Core logging method. It groups repeated messages per category and enqueues a formatted log line.
        /// </summary>
        /// <param name="category">The log category (e.g. "Measurement", "CFAR").</param>
        /// <param name="message">The log message text.</param>
        public static void Log(string category, string message)
        {
            // Group repeated messages to reduce log clutter.
            lock (groupingLock)
            {
                if (lastLogEntries.TryGetValue(category, out var entry))
                {
                    if (entry.Message == message)
                    {
                        // Same message: increase the count.
                        entry.Count++;

                        // If the last log was too recent, do not enqueue another line now.
                        if (DateTime.Now - entry.LastLogged < GroupInterval)
                        {
                            return;
                        }
                        else
                        {
                            // Enough time has elapsed: flush a summary line.
                            if (entry.Count > 1)
                            {
                                string summary = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [{category}] {entry.Message} (repeated {entry.Count} times)";
                                EnqueueLog(summary);
                            }
                            else
                            {
                                string logLine = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [{category}] {message}";
                                EnqueueLog(logLine);
                            }
                            entry.LastLogged = DateTime.Now;
                            entry.Count = 0;
                        }
                    }
                    else
                    {
                        // New message for this category: flush any pending summary for the previous message.
                        if (entry.Count > 1)
                        {
                            string summary = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [{category}] {entry.Message} (repeated {entry.Count} times)";
                            EnqueueLog(summary);
                        }
                        // Update with the new message.
                        lastLogEntries[category] = new LogEntry { Message = message, Count = 1, LastLogged = DateTime.Now };
                        string logLine = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [{category}] {message}";
                        EnqueueLog(logLine);
                    }
                }
                else
                {
                    // No previous entry for this category.
                    lastLogEntries[category] = new LogEntry { Message = message, Count = 1, LastLogged = DateTime.Now };
                    string logLine = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [{category}] {message}";
                    EnqueueLog(logLine);
                }
            }
        }

        /// <summary>
        /// Enqueues a log line to be written asynchronously.
        /// </summary>
        /// <param name="logLine">The formatted log line.</param>
        private static void EnqueueLog(string logLine)
        {
            logQueue.Enqueue(logLine);
        }

        /// <summary>
        /// Background task that dequeues log lines, writes them to file (with rotation), and stores them in memory.
        /// </summary>
        private static void ProcessQueue()
        {
            while (!cts.IsCancellationRequested)
            {
                if (logQueue.TryDequeue(out var logLine))
                {
                    try
                    {
                        // Rotate the log file if it exceeds the maximum size.
                        if (File.Exists(logFilePath))
                        {
                            var info = new FileInfo(logFilePath);
                            if (info.Length > MaxLogFileSizeBytes)
                            {
                                string archivePath = $"{Path.GetFileNameWithoutExtension(logFilePath)}_{DateTime.Now:yyyyMMdd_HHmmss}{Path.GetExtension(logFilePath)}";
                                File.Move(logFilePath, archivePath);
                            }
                        }
                        File.AppendAllText(logFilePath, logLine + Environment.NewLine);
                    }
                    catch (Exception ex)
                    {
                        // Fallback: write to Console if file logging fails.
                        Console.WriteLine("Error writing to log file: " + ex.Message);
                    }

                    // Save the log line in the in‑memory buffer.
                    lock (memoryLock)
                    {
                        debugMessages.Add(logLine);
                        if (debugMessages.Count > MaxMessagesInMemory)
                        {
                            debugMessages.RemoveAt(0);
                        }
                    }
                }
                else
                {
                    // No messages in the queue – wait briefly.
                    Thread.Sleep(50);
                }
            }
        }

        // --- Convenience methods for various log categories ---

        public static void LogMeasurement(string message) => Log("Measurement", message);
        public static void LogCFAR(string message) => Log("CFAR", message);
        public static void LogAssociation(string message) => Log("Association", message);
        public static void LogCandidate(string message) => Log("Candidate", message);
        public static void LogTrack(string message) => Log("Track", message);

        // --- Additional logging methods for debugging duplicate track issues ---
        public static void LogGating(string message) => Log("Gating", message);
        public static void LogTrackMerge(string message) => Log("TrackMerge", message);
        public static void LogDuplicate(string message) => Log("Duplicate", message);
    }
}
