from html.parser import HTMLParser

class TabParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack = []
        self.tabs = []

    def handle_starttag(self, tag, attrs):
        id_val = None
        for attr in attrs:
            if attr[0] == 'id':
                id_val = attr[1]
        
        self.stack.append({'tag': tag, 'id': id_val, 'line': self.getpos()[0]})
        
        if id_val and str(id_val).startswith('tab-'):
            depth = len(self.stack) - 1
            self.tabs.append({'id': id_val, 'line': self.getpos()[0], 'depth': depth})

    def handle_endtag(self, tag):
        for i in range(len(self.stack)-1, -1, -1):
            if self.stack[i]['tag'] == tag:
                popped = self.stack.pop(i)
                if popped['id'] and popped['id'].startswith('tab-'):
                    for t in self.tabs:
                        if t['id'] == popped['id'] and t['line'] == popped['line']:
                            t['end_line'] = self.getpos()[0]
                return

parser = TabParser()
with open("public/index.html", "r", encoding="utf-8") as f:
    parser.feed(f.read())

print("Tabs found:")
for t in parser.tabs:
    print(f"- {t['id']}: start {t['line']}, end {t.get('end_line', 'unclosed')}, depth {t['depth']}")
