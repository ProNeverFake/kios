import time


def bb_result_test(func):

    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        if result == False:
            pause = input("Failed. Press enter to continue")

        return result

    return wrapper


def execution_timer(func):
    """
    be careful with this since the return value is suppressed
    """

    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        func(*args, **kwargs)
        end = time.perf_counter()
        print(f"Execution time: {end - start} seconds")
        execution_duration = end - start
        return execution_duration

    return wrapper


def test_timer():
    @execution_timer
    def test_function():
        time.sleep(2)

    print(test_function())  # 2.0000000000000004


if __name__ == "__main__":
    test_timer()
