# from ros2cli.command import CommandExtension


# class KiosCommand(CommandExtension):
#     def add_arguments(self, parser, cli_name):
#         # The sub-command will be set as the "command" attribute of the parsed arguments
#         self._subparsers = parser.add_subparsers(
#             title='Subcommands', dest='command')

#     def main(self, *, parser, args):
#         if args.command:
#             # This will execute the main() function of the chosen sub-command module
#             args.main(args)
#         else:
#             print(f"Unknown command: {args.command}")


# # Importing the sub-commands here makes them discoverable when the CLI is used
# # ... import other sub-commands as you add them

# parser = KiosCommand._subparsers.add_parser('say', help='Print a message')
# parser.add_argument('message', type=str, help='Message to print')
# parser.set_defaults(main=say)

from ros2cli.command import CommandExtension
from kios_cli import commands  # Import implementations


class KiosCommand(CommandExtension):

    def add_arguments(self, parser, cli_name):
        subparsers = parser.add_subparsers(title='Subcommands', dest='command')

        say_parser = subparsers.add_parser('say', help='Print a message')
        say_parser.add_argument('message', type=str, help='Message to print')
        # Link to the "main" function in "say.py"
        say_parser.set_defaults(func=commands.say)

        # Add more sub-command parsers and link them to their implementation as needed.
        # another_command_parser = subparsers.add_parser(...)
        # another_command_parser.set_defaults(func=another_command.main)

    def main(self, *, parser, args):
        if hasattr(args, 'func'):
            args.func(args)
        else:
            print(f"Unknown command: {args.command}")
