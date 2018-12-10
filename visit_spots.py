import rospy
import yaml
import os

__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))


def get_spot(step):

    # Read information from yaml file
    with open(os.path.join(__location__, 'spots.yaml'), 'r') as stream:
        dataMap = yaml.load(stream)

    if step < len(dataMap):
        return dataMap[step]
    else:
        return {}


def get_num_spots():
    # Read information from yaml file
    with open(os.path.join(__location__, 'spots.yaml'), 'r') as stream:
        dataMap = yaml.load(stream)
    if dataMap:
        return len(dataMap)
    else:
        return 0
