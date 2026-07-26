from html.parser import HTMLParser

class MyHTMLParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack = []
        self.ids = set()
        self.duplicate_ids = []
        self.errors = []

    def handle_starttag(self, tag, attrs):
        self.stack.append((tag, self.getpos()))
        for attr in attrs:
            if attr[0] == 'id':
                val = attr[1]
                if val in self.ids:
                    self.duplicate_ids.append((val, self.getpos()))
                else:
                    self.ids.add(val)

    def handle_endtag(self, tag):
        # find matching start tag
        for i in range(len(self.stack)-1, -1, -1):
            if self.stack[i][0] == tag:
                # found matching
                del self.stack[i:]
                return
        self.errors.append(f"Unmatched end tag <{tag}> at line {self.getpos()[0]}")

parser = MyHTMLParser()
with open("public/index.html", "r", encoding="utf-8") as f:
    parser.feed(f.read())

print("Duplicate IDs:", parser.duplicate_ids)
print("Unmatched End Tags:", parser.errors)
print("Unclosed Start Tags:", [ (t, pos) for t, pos in parser.stack if t in ('div', 'section', 'main') ])
