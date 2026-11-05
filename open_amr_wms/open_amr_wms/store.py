"""SQLite persistence of the WMS: bins (SKU, box id), tasks (the whole lifecycle), the event log, counters.

Write-through: every change the WMS makes to a bin or a task is written at once (WAL journal, one connection), so a
WMS killed at any instant restarts with what it had told the fleet. ':memory:' for tests and stress runs.
"""
import json
import sqlite3

SCHEMA = """
CREATE TABLE IF NOT EXISTS bins (bin_id TEXT PRIMARY KEY, full INTEGER NOT NULL, sku TEXT, box_id TEXT,
                                 updated_s REAL);
CREATE TABLE IF NOT EXISTS tasks (task_id TEXT PRIMARY KEY, data TEXT NOT NULL, state TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, who TEXT, what TEXT, why TEXT,
                                   ref TEXT);
CREATE TABLE IF NOT EXISTS counters (key TEXT PRIMARY KEY, value INTEGER);
"""


class Store:
    def __init__(self, path=':memory:'):
        # one connection, shared by the ROS executor and the REST threads; every access holds the WMS lock (api.py)
        self.db = sqlite3.connect(path, isolation_level=None, check_same_thread=False)
        if path != ':memory:':
            self.db.execute('PRAGMA journal_mode=WAL')
            self.db.execute('PRAGMA synchronous=NORMAL')
        self.db.executescript(SCHEMA)

    # bins
    def bins(self):
        return {r[0]: dict(full=bool(r[1]), sku=r[2] or '', box_id=r[3] or '')
                for r in self.db.execute('SELECT bin_id, full, sku, box_id FROM bins')}

    def put_bin(self, bin_id, full, sku='', box_id='', t=0.0):
        self.db.execute('INSERT OR REPLACE INTO bins VALUES (?,?,?,?,?)', (bin_id, int(full), sku, box_id, t))

    # tasks
    def put_task(self, task_id, data, state):
        self.db.execute('INSERT OR REPLACE INTO tasks VALUES (?,?,?)', (task_id, json.dumps(data), state))

    def tasks(self, states=None):
        q = 'SELECT task_id, data, state FROM tasks'
        rows = self.db.execute(q).fetchall()
        return {r[0]: (json.loads(r[1]), r[2]) for r in rows if states is None or r[2] in states}

    # events
    def event(self, t, who, what, why='', ref=''):
        self.db.execute('INSERT INTO events (t, who, what, why, ref) VALUES (?,?,?,?,?)', (t, who, what, why, ref))

    def events(self, since_id=0, limit=1000):
        return [dict(id=r[0], t=r[1], who=r[2], what=r[3], why=r[4], ref=r[5]) for r in self.db.execute(
            'SELECT id, t, who, what, why, ref FROM events WHERE id > ? ORDER BY id LIMIT ?', (since_id, limit))]

    # counters
    def counter(self, key, default=0):
        r = self.db.execute('SELECT value FROM counters WHERE key=?', (key,)).fetchone()
        return r[0] if r else default

    def set_counter(self, key, value):
        self.db.execute('INSERT OR REPLACE INTO counters VALUES (?,?)', (key, int(value)))

    def close(self):
        self.db.close()
