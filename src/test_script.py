from rospit.rospit_xml import get_test_suite_from_xml_path
import sys, logging
parser = get_test_suite_from_xml_path("/home/openrtm/programming/rospit_ws/src/pits/xml/test_picks_and_places.xml", True)
logger = logging.getLogger("rospit default logger")
logger.setLevel(logging.INFO)
logger.addHandler(logging.StreamHandler(sys.stdout))
ts = parser.parse()
rep = ts.run(logger)
