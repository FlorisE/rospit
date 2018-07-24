""" A framework for running automated tests using external instrumentation """

from abc import ABCMeta, abstractmethod
from collections import namedtuple
from time import sleep
import future  # noqa:401, pylint: disable=W0611


class StatusMessages(object):
    PASSED = '\033[92m' + "PASSED" + '\033[0m'
    FAILED = '\033[91m' + "FAILED" + '\033[0m'


def status_message_from_boolean(value):
    if value:
        return StatusMessages.PASSED
    else:
        return StatusMessages.FAILED


class Runner(object):
    """
    Class for running test suites
    """

    def __init__(self):
        self.reports = []
        self.last_suite = None

    def run_suite(self, test_suite):
        """
        Runs the specified test_suite
        """
        self.last_suite = test_suite
        test_suite_report = test_suite.run()
        for test_case_report in test_suite_report.test_case_reports:
            status_message = status_message_from_boolean(test_case_report.passed())
            print("{}: {}".format(test_case_report.test_case.name, status_message))

    def run_all(self):
        """
        Autodetect test suites and runs them
        """
        pass


class TestSuite(object):
    """
    Container for test cases
    """
    __metaclass__ = ABCMeta

    def __init__(self, name=""):
        self.name = name
        self.test_cases = []

    def run(self):
        """
        Runs all the test cases in this test suite
        """
        print("Running test suite {}".format(self.name))
        test_case_reports = [tc.execute() for tc in self.test_cases]
        return TestSuiteReport(test_case_reports)


class TestCaseReport(object):
    def __init__(self, test_case, preconditions, invariants, postconditions, dependencies_met):
        self.test_case = test_case
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions
        self.dependencies_met = dependencies_met

    def passed(self):
        return all_conditions_nominal(self.preconditions) and \
               all_conditions_nominal(self.invariants) and \
               all_conditions_nominal(self.postconditions) and \
               self.dependencies_met


TestSuiteReport = namedtuple(
    "TestSuiteReport",
    ["test_case_reports"])


def all_conditions_nominal(conditions):
    if conditions is None:
        return True
    return all([c.nominal for c in conditions])


class TestCase(object):
    """
    Specifies a test case with preconditions, invariants and postconditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name="", preconditions=None, invariants=None,
                 postconditions=None, wait_for_preconditions=False,
                 sleep_rate=0.1, depends_on=None):
        self.name = name
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        if depends_on is None:
            depends_on = []
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions
        self.wait_for_preconditions = wait_for_preconditions
        self.sleep_rate = sleep_rate
        self.depends_on = depends_on
        self.report = None
        self.ran = False

    def verify_preconditions(self):
        """
        Evaluates all preconditions
        """
        return [e.evaluate(c) for c, e in self.preconditions]

    def verify_invariants(self, time):
        """
        Evaluates the invarians at time since starting
        """
        return [e.evaluate(c) for c, e in self.invariants]

    def verify_postconditions(self):
        """
        Evaluates all postconditions
        """
        return [e.evaluate(c) for c, e in self.postconditions]

    def all_preconditions_nominal(self, preconditions=None):
        if preconditions is None:
            preconditions = self.verify_preconditions()
        return all_conditions_nominal(preconditions)

    def execute(self):
        """
        Verifies the preconditions, runs the test, verifies the postconditions
        """
        print("Running test case {}".format(self.name))

        if not all([tc.report.passed() if tc.ran else False for tc in self.depends_on]):
            print("Dependencies have not been met, marking test case as failure")
            self.finish([], [], [], False)
            return self.report

        print("Verifying preconditions")
        if self.wait_for_preconditions:
            print("Wait for preconditions is enabled")
            preconditions_evaluation = self.verify_preconditions()
            while not self.all_preconditions_nominal(preconditions_evaluation):
                sleep(self.sleep_rate)
                preconditions_evaluation = self.verify_preconditions()
        else:
            print("Wait for preconditions is disabled")
            preconditions_evaluation = self.verify_preconditions()
            if not self.all_preconditions_nominal(preconditions_evaluation):
                self.finish(preconditions_evaluation, [], [])
                return self.report
        print("Running the body of the test")
        invariants = self.run()
        print("Verifying postconditions")
        postconditions_evaluation = self.verify_postconditions()
        self.finish(preconditions_evaluation, invariants, postconditions_evaluation)
        return self.report

    def finish(self, preconditions_evaluation, invariants, postconditions_evaluation, dependencies_met=True):
        self.ran = True
        self.report = TestCaseReport(self, preconditions_evaluation, invariants,
                                     postconditions_evaluation, dependencies_met)
        return self.report

    @abstractmethod
    def run(self):
        """
        Runs the test
        """
        pass


class Evaluation(object):
    """
    Evaluation produced by an Evaluator
    """
    def __init__(self, measurement, condition, nominal):
        self.measurement = measurement
        self.condition = condition
        self.nominal = nominal


class CompositeEvaluation(Evaluation):
    """
    Evaluation which consists of a list of evaluations for a single measurement
    """
    def __init__(self, measurement, condition, evaluations):
        Evaluation.__init__(
            self, measurement, condition,
            all([evaluation.nominal for evaluation in evaluations]))
        self.evaluations = evaluations


class TimestampEvaluationPair(object):
    """
    Pair of a timestamp and an evaluation
    """
    def __init__(self, timestamp, evaluation):
        self.timestamp = timestamp
        self.evaluation = evaluation


class ConditionEvaluatorPair(object):
    """
    Pair of a condition and an evaluator
    """
    def __init__(self, condition, evaluator):
        self.condition = condition
        self.evaluator = evaluator


class Condition(object):
    """
    Abstract base class for conditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, value, name=""):
        self.value = value
        self.name = name


class Evaluator(object):
    """
    Abstract base class for evaluators
    """
    __metaclass__ = ABCMeta

    def __init__(self, evaluator):
        self.evaluator = evaluator

    @abstractmethod
    def evaluate(self, condition, measurement):
        """
        Executes the evaluator by checking the measurement using the condition
        """
        pass

    def call_evaluator(self):
        if isinstance(self.evaluator, Sensor):
            return self.evaluator.sense()
        elif callable(self.evaluator):
            return self.evaluator()
        else:
            raise Exception("Evaluator is neither a sensor nor callable")


class Measurement(object):
    """
    Contains a measurement
    """
    __metaclass__ = ABCMeta


class Sensor(object):
    """
    Abstract base class for sensors
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        self.last_sensed = None

    @abstractmethod
    def sense_internal(self):
        """
        Reads the sensor value
        """
        pass

    def sense(self):
        self.last_sensed = self.sense_internal()
        return self.last_sensed
