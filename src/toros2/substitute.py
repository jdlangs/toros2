import click
import collections
import re

Sub = collections.namedtuple('Switch', ['pattern', 'replace', 'save'])

def apply_sub(data: str, sub: Sub, cache: dict, *, confirm=False):
    print('Checking for pattern: ', sub.pattern)
    def print_and_replace(m):
        matched = m.group(0)
        msgwidth = 20
        print(f"  {'pattern found':<{msgwidth}}: '{matched}'")
        repl = ''
        if matched in cache:
            repl = cache[matched]
            print(f"  {'cached replacement':<{msgwidth}}: '{repl}'")
        else:
            repl = sub.replace(m)
            print(f"  {'replacing with':<{msgwidth}}: '{repl}'")
        action = 'y'
        if confirm:
            action = click.prompt('yes/no/rewrite', default='y', type=click.Choice(['y','n','r']))
        if action == 'y':
            if sub.save:
                cache[matched] = repl
            return repl
        elif action == 'n':
            return None
        elif action == 'r':
            newrepl = click.prompt('  Input alternate replacement:', type=str)
            if sub.save:
                cache[matched] = newrepl

    data = re.sub(sub.pattern, print_and_replace, data)
    return data
