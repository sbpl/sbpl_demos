import json
import rospy
import os
import shutil
import time
from rospkg import RosPack
class GraspDatabase:

    def __init__(self, path):
        if path == None: 
            pkg = RosPack()
            self.db_file = os.path.join(pkg.get_path("sbpl_demos"), "data/grasp_database/roman_grasp_db.json")
        else:
            self.db_file = path
        self.db = self.__load__()
        
    def toJson(self):
        db = { key : [ { state : value.tolist() for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in self.db.iteritems()}
        return json.dumps(db, indent=4,sort_keys=True, separators=(',',': '))

    def __load__(self):
        if not os.path.exists(self.db_file):
            print "No db file detected. Creating a new one"
            return dict()
        with open(self.db_file,'r') as f:
            try:
                serialized = json.load(f)
            except:
                print("Unable to deserialize json (creating new dict):", sys.exc_info()[0])
                return dict()
            try:
                db = { key : [ { state : np.array(value) for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in serialized.iteritems()}
            except:
                print("Unable to parse json (creating new dict):", sys.exc_info()[0])
                return dict()
            return db
    def save(self):
        with open(self.db_file, 'w') as f:
            f.write(self.toJson())
            return self.db_file
    def backup(self, saveas=None):
        if saveas==None:
            saveas = "/tmp/db.json.backup."+ time.strftime("%d.%m.%y.%H.%M.%S")
        with open(saveas, 'w') as f:
            f.write(self.toJSON())
            return saveas
