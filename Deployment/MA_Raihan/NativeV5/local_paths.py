"""Small Python 2 path adapter; avoids installing a dependency on the robot."""
import io
import os


class Path(object):
    def __init__(self,value):self.value=str(value)
    def __str__(self):return self.value
    def __div__(self,other):return Path(os.path.join(self.value,str(other)))
    __truediv__=__div__
    def mkdir(self,parents=False,exist_ok=False):
        try:
            if parents:os.makedirs(self.value)
            else:os.mkdir(self.value)
        except OSError:
            if not (exist_ok and os.path.isdir(self.value)):raise
    def open(self,mode='r'):return open(self.value,mode)
    def write_text(self,text):
        with io.open(self.value,'w',encoding='utf-8') as handle:
            handle.write(text if isinstance(text,type(u'')) else text.decode('utf-8'))
    def rename(self,target):os.rename(self.value,str(target))
