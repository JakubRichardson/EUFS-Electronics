import pandas as pd

# Priority order for special components
priority_order = ['LV-Box', 'Accumulator', 'DC-Link', 'Chassis']


def normalize_point_pair(a, b):
    def foo(row):
        p1 = row[a].strip()
        p2 = row[b].strip()
        # If either point is in the priority list, apply custom ordering
        if p1 in priority_order or p2 in priority_order:
            p1_priority = priority_order.index(
                p1) if p1 in priority_order else float('inf')
            p2_priority = priority_order.index(
                p2) if p2 in priority_order else float('inf')
            # Lower index = higher priority
            if p1_priority < p2_priority:
                return f"{p1} <-> {p2}"
            elif p2_priority < p1_priority:
                return f"{p2} <-> {p1}"
            else:
                # Same priority, order alphabetically
                return f"{min(p1, p2)} <-> {max(p1, p2)}"
        else:
            # Default alphabetical order if neither is in priority list
            return f"{min(p1, p2)} <-> {max(p1, p2)}"
    return foo


def df_to_markdown_table(df):
    header = '| ' + ' | '.join(df.columns) + ' |\n'
    separator = '| ' + ' | '.join(['---'] * len(df.columns)) + ' |\n'
    rows = '\n'.join(['| ' + ' | '.join(str(cell)
                     for cell in row) + ' |' for row in df.values])
    return header + separator + rows + '\n'


def connection_sort_key(connection):
    parts = connection.split(' <-> ')
    priorities = [priority_order.index(
        p) if p in priority_order else float('inf') for p in parts]
    return (min(priorities), max(priorities), connection)


def group_signals_by_connection(file_path, output_file):
    # Read CSV with possible bad characters in headers
    df = pd.read_csv(file_path, encoding='utf-8')

    # Combine Start and End points
    start_nodes = df[df.columns[2]].str.strip()
    end_nodes = df[df.columns[3]].str.strip()

    all_nodes = pd.concat([start_nodes, end_nodes])
    unique_nodes = sorted(all_nodes.unique())
    print(unique_nodes)

    # Create normalized connection keys
    df['Connection'] = df.apply(normalize_point_pair(
        df.columns[2], df.columns[3]), axis=1)

    # Group by the normalized connection
    grouped = df.groupby('Connection')

    # Sort connections by priority
    sorted_connections = sorted(grouped.groups.keys(), key=connection_sort_key)

    # Print full rows for each group
    # for connection, group in grouped:
    #     print(f"\nConnection: {connection}")
    #     print(group.to_string(index=False))

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('# Grouped Signal Connections\n')
        for connection in sorted_connections:
            group = grouped.get_group(connection)
            f.write(f'\n## {connection}\n\n')
            f.write(df_to_markdown_table(group.drop(columns="Connection")))
            f.write('\n')


# Example usage
if __name__ == "__main__":
    file_path = "./Harnessing(Sheet1).csv"  # Replace with your CSV file path
    output_file = "grouped_signals.md"   # Output text file
    group_signals_by_connection(file_path, output_file)
