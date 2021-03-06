from lxml import etree

from rospit.framework import TestSuite, ConditionEvaluatorPair
from rospit.binary import BinaryCondition, BinaryConditionEvaluator, StaticBooleanEvaluator
from rospit.numeric import Limit, LowerLimitCondition, LowerLimitEvaluator, \
                           UpperLimitCondition, UpperLimitEvaluator, \
                           BothLimitsCondition, BothLimitsEvaluator, \
                           GreaterThanCondition, GreaterThanEvaluator, \
                           GreaterThanOrEqualToCondition, GreaterThanOrEqualToEvaluator, \
                           EqualToCondition, EqualToEvaluator, \
                           NotEqualToCondition, NotEqualToEvaluator, \
                           LessThanCondition, LessThanEvaluator, \
                           LessThanOrEqualToCondition, LessThanOrEqualToEvaluator
from rospit.declarative import DeclarativeTestCase, DummyStep
from rospit.ros import ServiceCall, get_subscriber, MessageReceivedEvaluator, \
                       Sleep, MessageEvaluator, ExecutionReturnedEvaluator, \
                       NumericMessageEvaluator, ROSTestSuite, MessageValue

NS = "{https://www.aist.go.jp/rospit}"
XSI_TYPE = "{http://www.w3.org/2001/XMLSchema-instance}type"

SCHEMA_PATH = "../xml/rospit.xsd"


def with_ns(element):
    return NS + element


def get_parser():
    loc = open(SCHEMA_PATH, "r")
    tree = etree.XML(loc.read())
    schema = etree.XMLSchema(tree)
    parser = etree.XMLParser(schema=schema)
    return parser


def get_test_suite_from_xml_path(path, validate=True):
    test_file = open(path, 'r').read()
    return get_test_suite_from_xml_string(test_file, validate)


def get_bool(value):
    if str(value) == "1":
        return True
    elif str(value) == "0":
        return False
    elif value == "true":
        return True
    elif value == "false":
        return False
    else:
        raise ValueError("value was not recognized as a valid boolean")


def get_test_suite_from_xml_string(xml, validate=True):
    if validate:
        tree = etree.fromstring(xml, get_parser())
    else:
        tree = etree.fromstring(xml)
    return Parser(tree)


class SubscriberFactory(object):
    def __init__(self, root):
        assert(root.tag == with_ns("TestSuite"))
        self.messages = dict()
        self.msg_value_subscribers = []
        self.msg_received_subscribers = []
        self.message_received_on = set()
        for test_case_elem in root.getchildren():
            self.parse_test_case(test_case_elem)

    def parse_test_case(self, elem):
        assert(elem.tag == with_ns("TestCase"))
        for phase_elem in elem.getchildren():
            self.parse_phase(phase_elem)

    def parse_phase(self, elem):
        name = elem.tag
        if name == with_ns("SetUp") or \
           name == with_ns("Run") or \
           name == with_ns("TearDown"):
            for step_elem in elem.getchildren():
                self.parse_step_elem(step_elem)
        elif name == with_ns("Preconditions") or \
             name == with_ns("Postconditions"):
            for condition_elem in elem.getchildren():
                self.parse_condition_elem(condition_elem)
        else:
            raise ValueError("Unexpected tag: {}".format(name))

    def parse_step_elem(self, elem):
        assert(elem.tag == with_ns("Step"))
        for message_elem in elem.getchildren():
            self.parse_message_elem(message_elem)

    def parse_message_elem(self, elem):
        assert(elem.tag == with_ns("Message"))
        for param_elem in elem.getchildren():
            self.parse_param_elem(param_elem)

    def parse_param_elem(self, elem):
        assert(elem.tag == with_ns("Parameter"))
        for value_elem in elem.getchildren():
            assert(value_elem.tag == with_ns("Value"))
            elem_type = value_elem.attrib[XSI_TYPE]
            if elem_type == "MessageValue":
                topic = value_elem.attrib['topic']
                msg_type = value_elem.attrib['type']
                self.msg_value_subscribers.append((topic, msg_type))

    def parse_condition_elem(self, elem):
        assert(elem.tag == with_ns("Precondition") or \
               elem.tag == with_ns("Postcondition"))
        for child in elem.getchildren():
            if child.tag == with_ns("Evaluator"):
                self.parse_evaluator_elem(child)

    def parse_evaluator_elem(self, elem):
        assert(elem.tag == with_ns("Evaluator"))
        elem_type = elem.attrib[XSI_TYPE]
        if elem_type == "MessageReceivedOnTopicEvaluator":
            topic = value_elem.attrib['topic']
            msg_type = value_elem.attrib['type']
            self.msg_received_subscribers.append((topic, msg_type))


class Parser(object):
    def __init__(self, root):
        self.root = root
        self.subs = SubscriberFactory(root)
        self.test_suite = None
        self.last_test_case = None
        self.current_test_case = None

    def parse(self):
        self.test_suite = ROSTestSuite(self.subs, self.root.attrib['name'])
        for child in self.root.getchildren():
            self.last_test_case = self.get_test_case_from_xml_element(child)
            self.test_suite.test_cases.append(self.last_test_case)
        return self.test_suite

    def get_test_case_from_xml_element(self, element):
        wait_for_preconditions = get_bool(element.attrib.get("wait_for_preconditions", "false"))
        depends_on_previous = get_bool(element.attrib.get("depends_on_previous", "false"))
        depends_on = [self.last_test_case] if depends_on_previous else None
        test_case = DeclarativeTestCase(name=element.attrib['name'], wait_for_preconditions=wait_for_preconditions, depends_on=depends_on)
        self.current_test_case = test_case
        for child in element.getchildren():
            name = child.tag
            if name == with_ns("SetUp"):
                for step in child.getchildren():
                    test_case.set_up_steps.append(self.get_step_from_xml_element(step))
            elif name == with_ns("Preconditions"):
                for condition in child.getchildren():
                    test_case.preconditions.append(self.get_condition_from_xml_element(condition))
            elif name == with_ns("Run"):
                for step in child.getchildren():
                    test_case.run_steps.append(self.get_step_from_xml_element(step))
            elif name == with_ns("Postconditions"):
                for condition in child.getchildren():
                    test_case.postconditions.append(self.get_condition_from_xml_element(condition))
            elif name == with_ns("TearDown"):
                for step in child.getchildren():
                    test_case.tear_down_steps.append(self.get_step_from_xml_element(step))
            else:
                raise Exception("Unexpected child")
        return test_case

    def get_condition_from_xml_element(self, element):
        for child in element.getchildren():
            if child.tag == with_ns("Condition"):
                condition = self.condition_factory(child)
            elif child.tag == with_ns("Evaluator"):
                evaluator = self.evaluator_factory(child)
            else:
                raise Exception("Unexpected child")
        return ConditionEvaluatorPair(condition, evaluator)


    def get_step_from_xml_element(self, element):
        elem_type = element.attrib[XSI_TYPE]
        if elem_type == "Dummy":
            return DummyStep()
        elif elem_type == "ServiceCall":
            parameters = self.get_message_from_service_call(element)
            save_result = get_bool(element.attrib.get('save_result', "false"))
            return ServiceCall(element.attrib['service'], element.attrib['type'], parameters, save_result=save_result)
        elif elem_type == "Sleep":
            return Sleep(element.attrib['duration'], element.attrib.get('unit', 'second'))
        else:
            raise Exception("Unidenfied step {}".format(elem_type))


    def get_message_from_service_call(self, element):
        children = element.getchildren()
        if len(children) == 1:
            message_elem = children[0]
            param_elems = message_elem.getchildren()
            if len(param_elems) != 0:
                params = self.get_param_from_elements(param_elems)
            else:
                params = dict()
            return params
        else:
            return None


    def get_param_from_elements(self, elements):
        params = dict()
        for param in elements:
            current_param = params
            value = self.get_value_from_element(param.getchildren()[0])
            name = param.attrib['name']
            name_parts = name.split('/')
            while len(name_parts) > 0:
                current_part = name_parts.pop(0)
                if len(name_parts) > 0:
                    current_param = current_param.setdefault(current_part, dict())
                else:
                    current_param[current_part] = value
        return params


    def get_value_from_element(self, element):
        elem_type = element.attrib[XSI_TYPE]
        if elem_type == "MessageValue":
            return MessageValue(element.attrib['topic'], element.attrib.get('field', None), self.test_suite)
        else:
            raise ValueError("Element is of unknown type")


    def condition_factory(self, element):
        elem_type = element.attrib[XSI_TYPE]
        attr = element.attrib
        name = attr.get("name", "")
        if elem_type == "Binary":
            return BinaryCondition(get_bool(attr['value']), name)
        elif elem_type == "GreaterThan":
            return GreaterThanCondition(float(attr['value']), name)
        elif elem_type == "GreaterThanOrEqualTo":
            return GreaterThanOrEqualToCondition(float(attr['value']), name)
        elif elem_type == "NotEqualTo":
            return NotEqualToCondition(float(attr['value']), name)
        elif elem_type == "EqualTo":
            return EqualToCondition(float(attr['value']), name)
        elif elem_type == "LessThanOrEqualTo":
            return LessThanOrEqualToCondition(float(attr['value']), name)
        elif elem_type == "LessThan":
            return LessThanCondition(float(attr['value']), name)
        elif elem_type == "UpperLimit":
            limit = Limit(float(attr['value']), get_bool(attr['inclusive']))
            return UpperLimitCondition(limit, name)
        elif elem_type == "LowerLimit":
            limit = Limit(float(attr['value']), get_bool(attr['inclusive']))
            return LowerLimitCondition(limit, name)
        elif elem_type == "BothLimits":
            lower_limit = Limit(float(attr['lower_limit_value']), get_bool(attr.get('lower_limit_inclusive', "true")))
            upper_limit = Limit(float(attr['upper_limit_value']), get_bool(attr.get('upper_limit_inclusive', "true")))
            return BothLimitsCondition(lower_limit, upper_limit, name)
        else:
            raise ValueError("Unexpected type {}".format(elem_type))


    def evaluator_factory(self, element):
        elem_type = element.attrib[XSI_TYPE]
        if elem_type == "StaticBooleanEvaluator":
            return StaticBooleanEvaluator(get_bool(element.attrib['value']))
        elif elem_type == "MessageReceivedEvaluator":
            return MessageReceivedEvaluator(element.attrib['topic'], element.attrib['type'], element.attrib.get('field', None))
        elif elem_type == "MessageEvaluator":
            return MessageEvaluator(element.attrib['topic'], element.attrib['type'], element.attrib.get('field', None))
        elif elem_type == "ExecutionReturnedEvaluator":
            field = element.attrib.get('field', None)
            return ExecutionReturnedEvaluator(self.current_test_case, field)
        elif elem_type == "NumericMessageEvaluator":
            return NumericMessageEvaluator(element.attrib['topic'], element.attrib['type'], element.attrib.get('field', None))
        else:
            raise ValueError("Unexpected type {}".format(elem_type))
