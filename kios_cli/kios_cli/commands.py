def say(args):
    if args.message:
        print(args.message)
    else:
        print("Usage: ros2 kios say \"<message>\"")
