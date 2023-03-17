import numpy as np
import argparse
import pickle
import lzma
import re


def save(states, file_name, compress):
    if compress:
        with lzma.open(file_name[:-4]+".xz", "wb") as f:
            pickle.dump(states, f, protocol=pickle.HIGHEST_PROTOCOL)
    else:
        with open(file_name[:-4]+".pickle", "wb+") as f:
            pickle.dump(states, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"saved file {file_name[:-4]}")

def parse_lines(lines, delay):
    for i, line in enumerate(lines):
        line = re.sub(r'state : \d* <--> c_', '', line)
        if delay == 0:
            line = re.sub(r' ->', '', line)
            line = re.sub(r'; ', '', line)
        else:
            w = line.split(', ->')
            w[0] = re.sub(',', '_', w[0])
            line = w[0]+w[1]
            line = re.sub(r'; ', '_', line)
        lines[i] = line
    return lines

def parse_shield(file_name, delay):
    states = {}
    with open(file_name, "r") as file:
        lines = file.readlines()
        lines = parse_lines(lines, delay)
        for l in lines:
            w = l.split(" ")
            action = [False, False, False]
            action[0] = True if "D" in w[1] else False
            action[1] = True if "S" in w[1] else False
            action[2] = True if "A" in w[1] else False
            states[w[0]] = action
    return states

def main():
    """parses the shield file into a dict that can be loaded in traffic_scenarios.py
       
       - split the shield txt into different delay files before processing
       - set the arguments according to the shield file
       - decompress the output file if compression is enabled

       shield file example:
       delay = 1
       <ego_dist>_<ego_velocity>_<other_dist>_<other_velocity>_<delay_actions> {<allowed_actions>}
       
       delay = 2
       <dist>_<velocity>_<delay_action[0]>_<delay_action[1]> {<actions>}
       
       actions = [D, S, A]   (bool)
    """
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
		'-f', '--file-name', 
		metavar='F',
        default='shield_pedestrian_v6_d2.txt',
		help='File name')
    argparser.add_argument(
        '--delay', 
        type=int, 
        default=2, 
        metavar='D', 
        help='How much delay is used for the shield')
    argparser.add_argument(
		'-p', '--path', 
		metavar='P',
		default='./shield/',
		help='Path for the folder of the files')
    argparser.add_argument(
		'--compress',
        action='store_true', 
        default=False,
		help='Should the file be compressed (this may need some time)')
    args = argparser.parse_args()
    if args.file_name is not None:
        args.file_name = args.path + args.file_name
    states = parse_shield(args.file_name, args.delay)
    save(states, args.file_name, args.compress)

if __name__ == '__main__':
    main()