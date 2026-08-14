import matplotlib.pyplot as plt

def plot_interval_differences(filename="timestamps.txt"):
    timestamps = []
    
    # 1. Parse the provided data file
    try:
        with open(filename, 'r') as f:
            current_sec = 0
            for line in f:
                line = line.strip()
                if line.startswith("sec:"):
                    current_sec = int(line.split(":")[1].strip())
                elif line.startswith("nanosec:"):
                    nanosec = int(line.split(":")[1].strip())
                    # Combine seconds and nanoseconds into a standard float
                    timestamps.append(current_sec + (nanosec * 1e-9))
    except FileNotFoundError:
        print(f"Error: Make sure you saved your data as '{filename}' in the same folder.")
        return

    if len(timestamps) < 3:
        print("Not enough data points to calculate interval differences.")
        return

    # 2. Calculate the base intervals (gaps) in milliseconds
    intervals_ms = [(timestamps[i] - timestamps[i-1]) * 1000 for i in range(1, len(timestamps))]

    # 3. Report the largest difference between the longest and shortest interval
    max_interval = max(intervals_ms)
    min_interval = min(intervals_ms)
    max_spread = max_interval - min_interval
    
    print("--- Interval Spread Statistics ---")
    print(f"Longest Interval: {max_interval:.2f} ms")
    print(f"Shortest Interval: {min_interval:.2f} ms")
    print(f"Largest Difference (Max - Min): {max_spread:.2f} ms")

    # 4. Calculate the differences between consecutive intervals
    # This represents how much the gap length changes from one frame to the next
    interval_diffs = [intervals_ms[i] - intervals_ms[i-1] for i in range(1, len(intervals_ms))]

    # 5. Plot the interval differences using a line chart
    plt.figure(figsize=(12, 6))
    
    # Line chart for the differences
    plt.plot(interval_diffs, marker='.', linestyle='-', color='purple', alpha=0.7, label='Interval Difference (ms)')
    
    # Highlight the largest absolute differences (Top 3 spikes)
    # Sorting by absolute magnitude to find the most severe jumps (positive or negative)
    largest_diffs = sorted(enumerate(interval_diffs), key=lambda x: abs(x[1]), reverse=True)[:3]
    
    for idx, val in largest_diffs:
        plt.plot(idx, val, marker='o', color='red')
        # Offset the text above or below the point depending on if it is a positive or negative spike
        y_offset = 10 if val > 0 else -15
        plt.annotate(f"{val:.2f} ms", 
                     (idx, val), 
                     textcoords="offset points", 
                     xytext=(0, y_offset), 
                     ha='center', 
                     color='red', 
                     weight='bold')

    # Formatting the chart
    plt.title('Frame-to-Frame Interval Differences')
    plt.xlabel('Interval Pair Sequence')
    plt.ylabel('Difference (Milliseconds)')
    plt.axhline(0, color='black', linestyle='--', alpha=0.5, label='Zero Change')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    
    # Display the plot
    plt.show()

if __name__ == "__main__":
    plot_interval_differences("timestamps.txt")