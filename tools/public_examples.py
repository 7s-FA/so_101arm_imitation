"""Replace site-specific addresses/accounts in exported text sources."""
import re

def examples(text):
    def ip(match):
        raw=match.group();parts=[int(x) for x in raw.split('.')]
        if any(x>255 for x in parts):return raw
        if parts[0]==10 or parts[:2]==[192,168] or (parts[0]==172 and 16<=parts[1]<=31):
            return '192.0.2.'+str(parts[3])
        return raw
    text=re.sub(r'(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])',ip,text)
    text=re.sub(r'/home/([^/\s]+)(?=/)', lambda m:'/home/'+(m[1] if m[1] in ('example_operator','example_robot','example_warehouse') else 'example_operator'), text)
    text=re.sub(r'\b([a-z_][a-z0-9_-]*)@(192\.0\.2\.)', lambda m:(m[1] if m[1] in ('example_operator','example_robot','example_warehouse') else 'example_robot')+'@'+m[2], text)
    return text
