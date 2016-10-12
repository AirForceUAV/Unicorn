#!/usr/bin/python  
# -*- coding=utf-8 -*-  
  
from library import root,get_location_metres

class Waypoint(object):
    def __init__(self,init_location,index):     
      file_path='waypoint.xml'
      self._root=root(file_path)[index]
      self._init=init_location
      self._number=0
      self._wp=self.init_points(init_location)
      self._count=len(self._wp)
      self._number=0

    def init_points(self,_init):
        result=[self._init]
        points=self._root.getchildren()
        if points is None:
            return []
        number=0        
        for point in points:
            _type=point.get('type')
            if _type=="0":
                result.append([self.child(point,0),self.child(point,1)])
            elif _type=="1":
                result.append(get_location_metres(result[number],self.child(point,2),self.child(point,3)))
            number+=1
        return result[1:]
    def get_remain_wp(self):
        return self._wp[self._number:]
    def get_all_wp(self):
        return self._wp
    def get_current_wp(self):
        return self._wp[self._number]
    def get_count(self):
        return self._count
    def add_number(self):
        self._number+=1
    def add_count(self):
        self._count+=1
    def minus_number(self):
        if self._number>=1:
            self._number-=1
    def minus_count(self):
        if self._count>=1:
            self._count-=1
    def child(self,point,index):
        return float(point[index].text)
    def write_xml(self, out_path):  
        
        self._root.write(out_path, encoding="utf-8",xml_declaration=True)  
                             
if __name__ == "__main__": 
    wp=Waypoint([39.25555,116.33333])
    print wp.get_points()
    wp.add_number()
    print wp.get_points()
    print wp.get_all_wp()
    print wp.get_currentwp()
      