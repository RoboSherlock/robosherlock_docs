# -*- coding: utf-8 -*-
"""
    sphinx.ext.todo
    ~~~~~~~~~~~~~~~
    Allow todos to be inserted into your documentation.  Inclusion of todos can
    be switched of by a configuration variable.  The todolist directive collects
    all todos of your project and lists them along with a backlink to the
    original location.
    :copyright: Copyright 2007-2011 by the Sphinx team, see AUTHORS.
    :license: BSD, see LICENSE for details.
"""

from docutils import nodes

from sphinx.locale import _
from sphinx.environment import NoUri
from sphinx.util.nodes import set_source_info
from sphinx.util.compat import Directive


import warnings

from docutils import nodes
from docutils.parsers.rst import Directive  # noqa

from docutils import __version__ as _du_version
docutils_version = tuple(int(x) for x in _du_version.split('.')[:2])


def make_admonition(node_class, name, arguments, options, content, lineno,
                    content_offset, block_text, state, state_machine):
    warnings.warn('make_admonition is deprecated, use '
                  'docutils.parsers.rst.directives.admonitions.BaseAdmonition '
                  'instead', DeprecationWarning, stacklevel=2)
    text = '\n'.join(content)
    admonition_node = node_class(text)
    if arguments:
        title_text = arguments[0]
        textnodes, messages = state.inline_text(title_text, lineno)
        admonition_node += nodes.title(title_text, '', *textnodes)
        admonition_node += messages
        if 'class' in options:
            classes = options['class']
        else:
            classes = ['admonition-' + nodes.make_id('todo')]
        admonition_node['classes'] += classes
    state.nested_parse(content, content_offset, admonition_node)
    return [admonition_node]


class colorbox_node(nodes.Admonition, nodes.Element): pass

class ColorBox(Directive):

    has_content = True
    required_arguments = 0
    optional_arguments = 1
    final_argument_whitespace = True
    option_spec = {}

    def run(self):
        env = self.state.document.settings.env
#        targetid = 'index-%s' % env.new_serialno('index')
#        targetnode = nodes.target('', '', ids=[targetid])
        ad = make_admonition(colorbox_node, self.name, [(self.arguments[0])], self.options,
                             self.content, self.lineno, self.content_offset,
                             self.block_text, self.state, self.state_machine)
        set_source_info(self, ad[0])
        
        return ad

def visit_todo_node(self, node):
    self.visit_admonition(node)

def depart_todo_node(self, node):
    self.depart_admonition(node)

def setup(app):
    app.add_node(colorbox_node,
                 html=(visit_todo_node, depart_todo_node),
                 latex=(visit_todo_node, depart_todo_node),
                 text=(visit_todo_node, depart_todo_node),
                 man=(visit_todo_node, depart_todo_node),
                 texinfo=(visit_todo_node, depart_todo_node))

    app.add_directive('colorbox', ColorBox)
