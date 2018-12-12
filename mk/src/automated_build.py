#!/usr/bin/python 
import sys
import os
import parsers.mod_mk_parser
import parsers.variable_list_parser


# Note that Jenkins defines the WORKSPACE env variable
if not os.environ.has_key("WORKSPACE"):
    os.environ["WORKSPACE"] = os.path.abspath(os.path.dirname(sys.argv[0]) + '../../..')
    print("Setting workspace to %s"%os.environ["WORKSPACE"])
	

linux_make = "make gen_make LINUX"
R5_make = "make gen_make TIR5" #TIR5_clean TIR5"
SD_make = "make gen_make SDFLIGHT" #SDFLIGHT_clean SDFLIGHT"
HEX_make = "make gen_make DSPAL" #DSPAL_clean DSPAL"
linux_clean = "echo clean" #"make clean"
R5_clean  = "echo clean" #"make TIR5_clean"
SD_clean  = "echo clean" #"make SDFLIGHT_clean"
HEX_clean = "echo clean" #"make DSPAL_clean"

deployment_list = [
    ("R5REF","TIR5", R5_make, R5_clean),
    ("R5REF","Linux", linux_make, linux_clean),
    ("HEXREF","DSPAL", HEX_make, HEX_clean),
    ("HEXREF","Linux", linux_make, linux_clean),
    ("SDREF","Snapdragon", SD_make, SD_clean),
    ("SDREF","Linux", linux_make, linux_clean),
    ("SIMREF","Linux", linux_make, linux_clean)
]

deployment_skip_list = []

make_failure = False
failed_deps = ""

# Make each deployment for the desired build targets:
for (deployment,label,target,clean_target) in deployment_list:
 
    if deployment in deployment_skip_list:
        print "!!! Skipping deployment %s\n" % deployment
        continue
     
    deployment_path = os.environ["WORKSPACE"] + "/" + deployment
     
    print "!!! Building Deployment %s\n" % deployment_path
    # os.chdir(deployment_path) # this causes issues w/ the Makefile CURDIR env variable
              
    make_str = "cd %s; %s" % (deployment_path, target)    
    print "!!! Executing command: %s\n" % make_str
    ret = os.system(make_str)
      
    if ret != 0:
        print "*** Deployment %s (%s) failed to compile!\n" % (deployment_path, label)
        make_failure = True
        failed_deps += "%s (%s) " % (deployment, label)
         
    # Clean the deployment, so the artifacts created don't effect the next deployment:
    make_str = "cd %s; %s" % (deployment_path, clean_target)    
    print "!!! Executing command: %s\n" % make_str
#     ret = os.system(make_str)
print "\nDeployments built:"
for (dep, label, target, clean_target) in deployment_list:
    print " %s (%s)" % (dep, label)

if make_failure:
    print "*** The following deployments failed: %s\n" % failed_deps
    sys.exit(-1)
# Must re-make the deployments for linux for the UTs, as they are cleaned above:
# else:
#     
#     for deployment in deployment_list.keys():
# 
#         if deployment in deployment_skip_list:
#             # print "!!! Skipping deployment %s\n" % deployment
#             continue
#         
#         deployment_path = os.environ["WORKSPACE"] + "/" + deployment
#         
#         (label, target, clean_target) = deployment_list[deployment]
#         
#         if label == "Linux":
#             make_str = "cd %s; %s" % (deployment, target)    
#             print "!!! Executing command to re-make deployment: %s\n" % make_str
#             ret = os.system(make_str)    

