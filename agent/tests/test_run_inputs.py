"""Explicit user-supplied graph inputs, without LLM or robot access."""
import json
import subprocess
import sys


def test_cli_passes_explicit_inputs_to_graph(tmp_path):
    graph=dict(schema='hei_agent.graph/v1',name='inputs',entry='done',nodes=[dict(id='done',kind='end',
        inputs=dict(status='succeeded',outputs=dict(value={'$ref':'inputs.value'})))],edges=[])
    path=tmp_path/'graph.json';path.write_text(json.dumps(graph))
    command=[sys.executable,'-m','hei_agent','--graph',str(path),'--inputs','{"value":0.025}','--trace-dir',str(tmp_path/'trace')]
    result=subprocess.run(command,text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['outputs']['value']==.025


def test_cli_rejects_conflicting_side_and_nonobject_inputs(tmp_path):
    for value in ('{"side":"left"}','[]'):
        result=subprocess.run([sys.executable,'-m','hei_agent','--side','right','--inputs',value],
                              text=True,capture_output=True,timeout=10)
        assert result.returncode==2 and ('inputs' in result.stderr.lower())
