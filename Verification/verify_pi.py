#!/usr/bin/env python3
"""
ROS 2 Bag Verifier
"""
import argparse, sqlite3, sys
from pathlib import Path

EXPECTED_TOPICS = {
    '/go1_controller/odom',
    '/go1_controller/imu/data',
    '/go1_controller/state',
    '/go1_controller/low_state',
    '/motor_states',
    '/joint_states',
    '/battery_state'
}

def verify_bag(bag_path):
    target_dir = Path(bag_path).expanduser().resolve()
    db_files = list(target_dir.glob('*.db3'))
    if not db_files:
        sys.exit("Error: No .db3 file found.")
    
    conn = sqlite3.connect(db_files[0])
    cursor = conn.cursor()
    
    cursor.execute("SELECT id, name, type FROM topics;")
    topics = {row[0]: {'name': row[1], 'type': row[2], 'count': 0} for row in cursor.fetchall()}
    
    cursor.execute("SELECT topic_id, COUNT(id) FROM messages GROUP BY topic_id;")
    for row in cursor.fetchall():
        if row[0] in topics: topics[row[0]]['count'] = row[1]
    conn.close()

    found = set()
    total = 0
    print(f"{'Topic Name':<40} | {'Type':<35} | {'Messages':<10}\n{'-'*90}")
    for _, data in sorted(topics.items(), key=lambda x: x[1]['name']):
        found.add(data['name'])
        total += data['count']
        print(f"{data['name']:<40} | {data['type']:<35} | {data['count']:<10}")
    
    missing = EXPECTED_TOPICS - found
    if missing or total == 0:
        print("\n❌ VERIFICATION FAILED: Missing topics or empty bag.")
        [print(f" - {m}") for m in missing]
    else:
        print("\n✅ VERIFICATION PASSED: All critical topics populated.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=str)
    args = parser.parse_args()
    verify_bag(args.bag)