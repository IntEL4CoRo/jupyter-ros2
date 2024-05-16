import os
from IPython.display import display, HTML
from sidecar import Sidecar

# Display remote desktop on sidecar tab
def display_desktop():
    try:
        JUPYTERHUB_USER = os.environ['JUPYTERHUB_USER']
    except KeyError:
        JUPYTERHUB_USER = None
    url_prefix = f"/user/{JUPYTERHUB_USER}" if JUPYTERHUB_USER is not None else ''
    remote_desktop_url = f"{url_prefix}/desktop"
    sc = Sidecar(title='Desktop')
    with sc:
        # The inserted custom HTML and CSS snippets are to make the tab resizable
        display(HTML(f"""
            <style>
            body.p-mod-override-cursor div.iframe-widget {{
                position: relative;
                pointer-events: none;

            }}

            body.p-mod-override-cursor div.iframe-widget:before {{
                content: '';
                position: absolute;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background: transparent;
            }}
            </style>
            <div class="iframe-widget" style="width: calc(100% + 10px);height:100%;">
                <iframe src="{remote_desktop_url}" width="100%" height="100%"></iframe>
            </div>
        """))