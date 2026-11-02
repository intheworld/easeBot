"""Generated example variants must inherit shared nodes and match shipped JSON."""
from pathlib import Path
import importlib.util
import json
from hei_agent.graph import Graph

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('generate_examples',ROOT/'scripts/generate-agent-examples.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_shipped_graph_variants_match_canonical_sources():
    variants=module.generated()
    assert len(variants)==10
    for name,value in variants.items():
        assert value==json.loads((module.EXAMPLES/name).read_text()),name
        Graph.from_dict(value)


def test_shared_argument_changes_propagate_to_tilt_variant():
    recipes=json.loads(module.RECIPES.read_text())['variants']
    base=json.loads((module.EXAMPLES/'right_pick_place_region.json').read_text())
    node=next(n for n in base['nodes'] if n['id']=='place_plan')
    node['inputs']['arguments']['approach_height_m']=.07
    result=module.derive(base,recipes['right_pick_place_region_tilt.json'])
    derived=next(n for n in result['nodes'] if n['id']=='place_plan')
    assert derived['inputs']['arguments']['approach_height_m']==.07
    assert derived['inputs']['skill']=='plan_object_placement_with_tilt'
