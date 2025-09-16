import dash
from dash import html, dcc, Output, Input
import dash_cytoscape as cyto
import threading
from collections import defaultdict


class GraphUI:
  def __init__(self, port=8050, interval_ms=1000):
    self.elements = {}
    self.index: dict[tuple[str,str],int]
    self.app = dash.Dash(__name__, suppress_callback_exceptions=True)

    self.app.layout = html.Div([
      html.H3("Programmatic Cytoscape Graph"),
      cyto.Cytoscape(
        id='cytoscape',
        layout={'name': 'cose'},
        style={'width': '100%', 'height': '600px'},
        elements=[],
        stylesheet=[
          {
            "selector": "node",
            "style": {"content": "data(label)"}
          },
          {
            "selector": "edge",
            "style": {
              "label": "data(label)",
              "width": "mapData(weight, 1, 10, 1, 10)",
              "font-size": "12px",
              "text-rotation": "autorotate",
              "curve-style": "bezier"
            }
          }
        ]
      ),
      dcc.Interval(id="refresh", interval=interval_ms, n_intervals=0)
    ])

    # update callback
    self.app.callback(
      Output('cytoscape', 'elements'),
      Input('cytoscape', 'id')  # dummy trigger
    )(self._update_elements)

    self.port = port

  # -----------------
  # Public API
  # -----------------
  def set_node(self, nid):
    id = nid
    self.elements[id] = {"data": {"id": nid, "label": nid}}
    self.refresh()

  def set_edge(self, src, dst, label="", weight=1):
    id = f"{src}--{dst}"
    self.elements[id] = {"data": {
      "id": id,
      "source": src,
      "target": dst,
      "label": label,
      "weight": weight,
    }}
    self.refresh()

  # -----------------
  # Internal
  # -----------------
  def _update_elements(self, _):
    res = list(self.elements.values())
    print(res)
    return res

  def refresh(self):
    # force Cytoscape to refresh by touching the layout
    self.app._cached_layout = None
    print(self.elements)

  def run(self, debug=True, use_thread=False):
    if use_thread:
      t = threading.Thread(
        target=lambda: self.app.run(debug=debug, port=self.port, use_reloader=False),
        daemon=True
      )
      t.start()
    else:
      self.app.run(debug=debug, port=self.port)


if __name__ == "__main__":
  g = GraphUI()
  g.add_node("A")
  g.add_node("B")
  g.add_edge("A", "B")
  g.run()
