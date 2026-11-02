#!/usr/bin/env python3
"""Generate compatible graph variants from canonical examples; never execute a graph."""
from copy import deepcopy
import argparse
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
EXAMPLES=ROOT/'agent/src/hei_agent/examples'
RECIPES=Path(__file__).with_name('agent-example-variants.json')


def derive(source,recipe):
    value=deepcopy(source)
    value['nodes']={node['id']:node for node in value['nodes']}
    for path in recipe.get('remove',[]):
        parts=path.split('/');parent=value
        for part in parts[:-1]:parent=parent[part]
        del parent[parts[-1]]
    for path,item in recipe.get('set',{}).items():
        parts=path.split('/');parent=value
        for part in parts[:-1]:parent=parent.setdefault(part,{})
        parent[parts[-1]]=deepcopy(item)
    value['nodes']=[value['nodes'][key] for key in recipe['nodes']]
    if 'edges' in recipe:value['edges']=deepcopy(recipe['edges'])
    return value


def generated():
    recipes=json.loads(RECIPES.read_text())['variants'];cache={};visiting=set()
    def load(name):
        if name in cache:return cache[name]
        if Path(name).name!=name or not name.endswith('.json'):raise ValueError('Invalid example name')
        if name in visiting:raise ValueError('Cyclic graph variant dependency')
        visiting.add(name)
        if name in recipes:value=derive(load(recipes[name]['source']),recipes[name])
        else:value=json.loads((EXAMPLES/name).read_text())
        visiting.remove(name);cache[name]=value
        return value
    return {name:load(name) for name in recipes}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='Reject edited/outdated generated examples without writing')
    args=parser.parse_args();changed=[]
    for name,value in generated().items():
        path=EXAMPLES/name
        if not path.exists() or json.loads(path.read_text())!=value:
            changed.append(name)
            if not args.check:path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    if args.check and changed:raise SystemExit('Regenerate graph variants: '+', '.join(changed))
    print(f'{len(generated())} graph variants '+('checked' if args.check else f'generated ({len(changed)} updated)'))


if __name__=='__main__':main()
