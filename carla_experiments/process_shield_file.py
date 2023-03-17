import argparse
import parse_shield


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
		'-f', '--file-name',
		metavar='F',
        default='shield_pedestrian_v9.txt',
		help='File name')
    args = argparser.parse_args()
    shield_name = args.file_name
    with open(f'{shield_name}.txt') as fp:
        lines = fp.readlines()

    current_delta = 0
    current_list = []
    for line in lines:
        if 'delay delta' in line:

            if current_delta%2 == 0:
                file_name = f'{shield_name}_d{int(current_delta/2)}.txt'
                with open(file_name, 'w') as fp:
                    for state in current_list:
                        fp.write(state)
                states = parse_shield.parse_shield(file_name, current_delta/2)
                parse_shield.save(states, file_name, False)
            current_delta = int(line.split('delta = ')[-1].split(':')[0])
            current_list = []

        if 'state :' in line:
            current_list.append(line)
    if current_delta%2 == 0:
                file_name = f'{shield_name}_d{int(current_delta/2)}.txt'
                with open(file_name, 'w') as fp:
                    for state in current_list:
                        fp.write(state)
                states = parse_shield.parse_shield(file_name, current_delta/2)
                parse_shield.save(states, file_name, False)

if __name__=='__main__':
    main()
