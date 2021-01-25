import os
os.environ['WEBOTS_HOME'] = r'C:\Users\tobyb\AppData\Local\Programs\Webots\\'
from collector_bot import Collector

coll = Collector()
coll.run()
