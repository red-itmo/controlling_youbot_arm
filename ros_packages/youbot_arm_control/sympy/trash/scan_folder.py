import os

def getDataFileNames(path):
    for dirname, dirnames, filenames in os.walk(path):
        return filenames

if __name__ == '__main__':
    path = '/media/data/evo/robotics_report/ros_packages/youbot_arm_control/calculations/data_for_identification/bigs/raw'
    fileNames = getDataFileNames(path)
    print(fileNames)