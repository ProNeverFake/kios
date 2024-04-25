"""
for handling the exceptions in the code and stablize the process.
"""


class ParsingException(Exception):
    def __init__(self, message):
        super().__init__(message)


class FormatException(Exception):
    def __init__(self, message):
        super().__init__(message)


class RuntimeException(Exception):
    def __init__(self, message):
        super().__init__(message)


class CommunicationException(Exception):
    def __init__(self, message):
        super().__init__(message)
