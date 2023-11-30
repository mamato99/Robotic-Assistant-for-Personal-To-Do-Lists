from flask import Flask, render_template, request
import sqlite3
import os

import sys
sys.path.append('config')

from config import *

app = Flask(__name__)

@app.route("/")
def hello_world():
    return render_template("home.html")

@app.route("/show", methods=['GET','POST'])
def show():
        
    con = sqlite3.connect(DB_PATH)
    cursor = con.cursor()
    print("Successfully Connected to SQLite")
    
    user=request.args.get('user')
    category=request.args.get('category')
    
    if category != "all": 
        sqlite_search_query = "select tag, activity, deadline, reminder from todolist where user= ? and category = ? "
        cursor.execute(sqlite_search_query, (user, category, ))
        page = "index1.html"
    else: 
        sqlite_search_query = "select category from todolist where user= ? "
        cursor.execute(sqlite_search_query, (user, ))
        page = "index2.html"
        
    data = cursor.fetchall()
    con.close()
    
    return render_template(page, data=set(data))

if __name__=="__main__":
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("--debug", "-d", dest="debug_mode", action = "store_true", default=False)
    (options, args) = parser.parse_args()
    
    app.run(host="0.0.0.0",debug=options.debug_mode)
