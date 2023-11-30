#!/usr/bin/env python3
from datetime import datetime,timedelta
from std_msgs.msg import String
import sqlite3 as sql

CREATE_TABLE_QUERY = """CREATE TABLE todolist(
	tag INTEGER PRIMARY KEY AUTOINCREMENT,
  	user varchar(255) NOT NULL,
	category varchar(255),
	activity varchar(255),
	deadline datetimeoffset,
    reminder boolean,
	unique(user,activity,category)
);
"""

class Reminder():
    
    def __init__(self, db_path, username:str = None, verbose:int = 0):
        self.db_path = db_path
        self.username = username if username is None else username.lower()
        self.verbose = verbose 

    def get_username(self) -> str:
        if self.username is None:
            raise RuntimeError
        return self.username
    
    def set_username(self, username:str):
        self.username = username.lower()

    def _get_connetion(self):
        con = sql.connect(self.db_path)
        cur = con.cursor()
        try:
            cur.execute("SELECT * FROM todolist")
        except sql.OperationalError as e:
            print("[REMINDER] Table creation...")
            cur.execute(CREATE_TABLE_QUERY)

        return con, cur

    def check_deadline(self):
        con, cur = self._get_connetion()
        expiring = dict()
        query = f"select tag,category,activity,deadline from todolist where reminder=true and user=?"
        if self.verbose > 1: print('[REMINDER] query',query)
        res = cur.execute(query,(self.get_username(),))
        tmp = res.fetchall()
        if self.verbose > 1: print('[REMINDER] query rsp:',tmp)
        if len(tmp) == 0:
            return None
        for el in tmp:
            deadline = datetime.fromisoformat(el[3])
            if self.verbose > 0: print("[REMINDER] dealine:",deadline,',',el)
            if (deadline.replace(tzinfo=None) - timedelta(hours=1) < datetime.now()):
                if el[1] in expiring:
                    expiring[el[1]] += [el[2]]
                else:
                    expiring[el[1]] = [el[2]]
                query = f"update todolist set reminder= 0 where tag = ?"
                res = cur.execute(query,(el[0],))
        con.commit()
        con.close()
        return expiring
    
    def remind_me(self):
        resp = self.check_deadline()
        
        if resp is None:
            return None
        
        to_say = ''
        for category in resp:
            to_say += f'In category {category} are expiring the following activities\n'
            for activity in resp[category]:
                to_say += activity + '\n'
            to_say += '\n'
        
        return to_say

# if __name__ == '__main__':
#     import pathlib
#     import rospy
#     DB_PATH=str(pathlib.Path(__file__).parent.absolute()) + "/../../rasa_ros/database.db"
#     r = Reminder(DB_PATH, 'francesco')
#     # print(r.check_deadline())
#     print(r.remind_me())