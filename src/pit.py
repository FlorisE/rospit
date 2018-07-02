""" A framework for running automated tests using external instrumentation """
from abc import ABCMeta, abstractmethod
from collections import namedtuple
import future  # noqa:401, pylint: disable=W0611


class Runner(object):
    """
    Class for running test suites
    """

    def __init__(self):
        self.reports = []

    def run_suite(self, test_suite):
        """
        Runs the specified test_suite
        """
        report = test_suite.run()
        self.reports.append(report)

    def run_autodetect(self):
        """
        Autodetect test suites and runs them
        """
        pass


class TestSuite(object):
    """
    Container for test cases
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name
        self.test_cases = []

    def run(self):
        """
        Runs all the test cases in this test suite
        """
        return [test_case.execute() for test_case in self.test_cases]


ExecutionResults = namedtuple(
    "ExecutionResults",
    ["preconditions", "run_result", "postconditions"])


class TestCase(object):
    """
    Specifies a test case with preconditions, invariants and postconditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name, preconditions=None, invariants=None,
                 postconditions=None):
        self.name = name
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions

    def verify_preconditions(self):
        """
        Evaluates all preconditions
        """
        return [c.evaluate() for c in self.preconditions]

    def verify_invariants(self, time):
        """
        Evaluates the invarians at time since starting
        """
        return [c.evaluate(time) for c in self.invariants]

    def verify_postconditions(self):
        """
        Evaluates all postconditions
        """
        return [c.evaluate() for c in self.postconditions]

    def execute(self):
        """
        Verifies the preconditions, runs the test, verifies the postconditions
        """
        preconditions_evaluation = self.verify_preconditions()
        run_result = self.run()
        postconditions_evaluation = self.verify_postconditions()
        return ExecutionResults(preconditions_evaluation, run_result,
                                postconditions_evaluation)

    @abstractmethod
    def run(self):
        """
        Runs the test
        """
        pass


Report = namedtuple(
    "Report",
    ["test_case", "passed", "preconditions", "invariants", "postconditions"])


ConditionEvaluationResults = namedtuple(
    "ConditionEvaluationResults",
    ["failed", "passed", "violations"])


Violation = namedtuple(
    "Violation",
    ["time", "conditions"])


class Condition(object):
    """
    Superclass for conditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name

    @abstractmethod
    def verify(self, time):
        """
        Verifies the condition
        """
        pass


class Measurable(object):
    """
    Mixin to make a measurement
    """

    @abstractmethod
    def measure_at(self, time):
        """
        Measures at time
        """
        pass


class InCategoryCondition(Condition, Measurable):
    """
    Condition that a value is within a list of accepted values
    """
    __metaclass__ = ABCMeta

    def __init__(self, name, categories=None):
        Condition.__init__(self, name)
        if categories is None:
            categories = set()
        self.categories = categories

    def verify(self, t):
        """
        Measure the category and verify whether it is allowed
        """
        return self.measure_at(t) in self.categories

    @abstractmethod
    def measure_at(self, time):
        """
        Measures at time
        """
        pass


class BinaryCondition(Condition, Measurable):
    """
    Condition that is either True or False
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        Condition.__init__(self, name)

    def verify(self, t):
        return self.measure_at(t) == self.expect_at(t)

    @abstractmethod
    def expect_at(self, time):
        """
        Get the expected value at the specified time
        """
        pass

    @abstractmethod
    def measure_at(self, time):
        """
        Measures at time
        """
        pass


class NumericCondition(Condition, Measurable):
    """
    A condition for a numeric function
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, name, lower_limit_is_inclusive=True,
            upper_limit_is_inclusive=True):
        Condition.__init__(self, name)
        self.lower_limit_is_inclusive = lower_limit_is_inclusive
        self.upper_limit_is_inclusive = upper_limit_is_inclusive

    @abstractmethod
    def lower_limit_at(self, time):
        """
        Sets an lower bound at the specified time
        """
        pass

    @abstractmethod
    def upper_limit_at(self, time):
        """
        Sets an upper bound at the specified time
        """
        pass

    def verify(self, t):
        measured = self.measure_at(t)
        lower_limit = self.lower_limit_at(t)
        upper_limit = self.upper_limit_at(t)
        if self.upper_limit_is_inclusive:
            lower_than_upper_limit = measured <= upper_limit
        else:
            lower_than_upper_limit = measured < upper_limit
        if self.lower_limit_is_inclusive:
            higher_than_lower_limit = measured >= lower_limit
        else:
            higher_than_lower_limit = measured > lower_limit
        return lower_than_upper_limit and higher_than_lower_limit

    @abstractmethod
    def measure_at(self, time):
        """
        Measures at time
        """
        pass
