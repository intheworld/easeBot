"""A partial physical result is evidence, never a successful graph outcome."""
import asyncio
import pytest
from types import SimpleNamespace
from hei_agent_bridge.skills import SkillSet,SkillFailure,MockSkillBackend


@pytest.mark.parametrize('kind',['valid','invalid_schema','cleanup_failure','ordinary_error'])
def test_partial_result_requires_declared_schema_and_confirmed_cleanup(kind):
    async def run():
        backend=MockSkillBackend()
        async def handler(args,invocation):
            if kind=='ordinary_error':raise RuntimeError('ordinary failure')
            raise SkillFailure('Release confirmed; retreat incomplete',{'released':True} if kind!='invalid_schema' else {'other':True})
        capabilities=SimpleNamespace(register=lambda skills:skills.register('partial','test',{}, {'released':{'type':'boolean'}},handler,motion=True))
        skills=SkillSet({'backend':'mock','named_poses':{},'gripper_max_gap_m':.1},backend,cameras=capabilities)
        if kind=='cleanup_failure':
            async def bad(invocation):raise RuntimeError('stop unconfirmed')
            backend.cancel=bad
        result=await skills.invoke('partial',{},invocation='one',revision=skills.revision)
        assert result['status']=='failed' and result['outcome']=='failure'
        assert result['outputs']==({'released':True} if kind=='valid' else {})
        assert result['cleanup_complete']==(kind!='cleanup_failure')
        assert skills.blocked==(kind=='cleanup_failure')
    asyncio.run(run())
