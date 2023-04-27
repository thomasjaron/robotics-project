import subprocess
import os
import time
import sys
import signal
#back2backe
#ezezasdfghjk


def group_mambers(report_file):
    # Open the file and read its contents
    with open('/home/evaluator/first_project/src/first_project/info.txt', 'r') as f:
        lines = f.readlines()

    # Iterate through the lines and print the name and surname
    print ("Group members:")
    report_file.write("Group members:\n")
    for line in lines:
        # Split the line into its three components (id, name, surname)
        components = line.strip().split(';')
        if len(components) != 3:
            continue
        id, name, surname = components

        # Print the name and surname in reverse order
        print(f"{surname}, {name}, id:{id}")
        report_file.write (f"{surname}, {name}, id:{id}\n")
        



def is_node_publishing(node_name, topic_name):
    # Execute the rostopic command and capture its output
    cmd = ['rostopic', 'info', topic_name]
    result = subprocess.run(cmd, capture_output=True, text=True)
    # Parse the output to extract the publishers list
    publishers = []
    is_publishers_line = False
    for line in result.stdout.split('\n'):
      if line.startswith('Publishers:'):
        is_publishers_line = True
      elif line.startswith('Subscribers:'):
        is_publishers_line = False
      elif is_publishers_line:
        for element in line.strip().split():
          publishers.append(element)

    # Check if the specified node is in the publishers list
    return node_name in publishers

#create report file

report_file = open("/home/evaluator/first_project/src/first_project/report.txt", "w+")
report_file.write("Automatically generated report\n\n")

#first step, check the group mambers
group_mambers(report_file)
report_file.write ("\nCode test:\n")
# Execute the command "catkin_make"
result = subprocess.run(['cd /home/evaluator/first_project ; source /home/evaluator/.bashrc; catkin_make'], capture_output=True, text=True, cwd="/home/evaluator/first_project", shell = True)

# Check if the command was successful
if result.returncode == 0:
    print("Compile executed successfully.")
    report_file.write ("1.Package succesfully compiled\n")
else:
    print("Compile failed with the following output:")
    report_file.write ("1.Package compile error\n")
    print(result.stdout)
    sys.exit (1)
try:
  roscore_process = subprocess.Popen(['bash', '-c', 'source /home/evaluator/.bashrc; roscore'],stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  time.sleep(2)
  print("Roscore started successfully.")
except subprocess.CalledProcessError as e:
    print("Roscore failed with the following output:")
    print (e.output)
    sys.exit(1) 

try:
  publisher_process = subprocess.Popen(['bash', '-c', 'source /home/evaluator/.bashrc; source /home/evaluator/first_project/devel/setup.bash; rospack profile; rosrun first_project odom_node'],stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  time.sleep(2)
  print ("pub started")
  report_file.write ("2.Publisher started succesfully\n")
except subprocess.CalledProcessError as e:
    print("publisher failed with the following output:")
    report_file.write ("1.Publisher start error\n")
    print (e.output)
    sys.exit(1) 

time.sleep(5)


if (is_node_publishing ( '/odom_node', '/odometry')):
  print ("publisher is working")
  report_file.write ("3.Publisher is publishing data\n")
else:
  print ("publisher is not working")
  report_file.write ("3.Publisher is not publishing data\n")


time.sleep(5)

report_file.close()
#publisher_process.terminate()
os.killpg(os.getpgid(publisher_process.pid), signal.SIGTERM)
time.sleep(2)
os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)
#roscore_process.terminate() 
time.sleep(2)
print ("sample test completed")

