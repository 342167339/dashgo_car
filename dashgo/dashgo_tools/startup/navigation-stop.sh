#!/bin/bash

ProgName="navigation_demo_web.launch"
num=$(ps aux|grep ${ProgName} |grep -v grep|wc -l)

if [ $num -gt 0 ] 
then 
   ps aux|grep ${ProgName} |grep -v grep|awk '{print  $2}'|xargs kill
   echo "$ProgName is shutting down!"
else
   echo "$ProgName  isn't exist!"
fi

