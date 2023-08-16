from kios_cli.kios import KiosCommand


def say(args):
    if args.message:
        print(args.message)
    else:
        print("Usage: ros2 kios say \"<message>\"")


parser = KiosCommand._subparsers.add_parser('say', help='Print a message')
parser.add_argument('message', type=str, help='Message to print')
parser.set_defaults(main=say)
