#!/usr/bin/evn python
#coding:utf-8

class config(object):
    def __init__(self):
        try: 
            import xml.etree.cElementTree as ET 
        except ImportError: 
            import xml.etree.ElementTree as ET 
        import sys    
        try: 
            tree = ET.parse("common.xml")     #open xml 
            self.root = tree.getroot()        
        except Exception, e: 
            print "Error:cannot parse file:common.xml." 
            sys.exit(1)
    def isInt(self,x):
        try:
            x=int(x)
            return isinstance(x,int)
        except ValueError:
            return False
    def subElement(self,root,param1,param2):
        value=self.root[param1][param2].get('value')
        if self.isInt(value) is True:
            return int(value)
        else:
            return value
    def get_cloud(self):
        return [self.subElement(self.root,0,1),self.subElement(self.root,0,2)]
    def get_AIL(self):
        return [self.subElement(self.root,1,1)-1,self.subElement(self.root,1,2),self.subElement(self.root,1,3),self.subElement(self.root,1,4),self.subElement(self.root,1,5)]
    def get_ELE(self):
        return [self.subElement(self.root,2,1)-1,self.subElement(self.root,2,2),self.subElement(self.root,2,3),self.subElement(self.root,2,4),self.subElement(self.root,2,5)]
    def get_THR(self):
        return [self.subElement(self.root,3,1)-1,self.subElement(self.root,3,2),self.subElement(self.root,3,3),self.subElement(self.root,3,4),self.subElement(self.root,3,5)]
    def get_RUD(self):
        return [self.subElement(self.root,4,1)-1,self.subElement(self.root,4,2),self.subElement(self.root,4,3),self.subElement(self.root,4,4),self.subElement(self.root,4,5)]
    def get_Mode(self):
        return [self.subElement(self.root,5,1)-1,self.subElement(self.root,5,2),self.subElement(self.root,5,3),self.subElement(self.root,5,4)]
    def get_PIT(self):
        return [self.subElement(self.root,6,1)-1,self.subElement(self.root,6,2),self.subElement(self.root,6,3),self.subElement(self.root,6,4),self.subElement(self.root,6,5)]
    def get_mavlink(self):
        return [self.subElement(self.root,7,1),self.subElement(self.root,7,2)]
    def get_GPS(self):
        return [self.subElement(self.root,8,1),self.subElement(self.root,8,2)]
    def get_compass(self):
        return [self.subElement(self.root,9,1),self.subElement(self.root,9,2)]
if __name__=="__main__":
    config=config()
    print config.get_cloud()
    print config.get_AIL()
    print config.get_ELE()
    print config.get_THR()
    print config.get_RUD()
    print config.get_Mode()
    print config.get_PIT()
    print config.get_mavlink()
    print config.get_GPS()
    print config.get_compass()

