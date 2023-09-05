from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QVBoxLayout
from rviz import bindings as rviz


def create_map_widget(rviz_path):
    '''.'''
    mapWidget = QWidget()

    # Create the RViz visualization frame.
    mapWidget.frame = rviz.VisualizationFrame()
    mapWidget.frame.setSplashPath("")
    mapWidget.frame.initialize()

    reader = rviz.YamlConfigReader()
    config = rviz.Config()
    reader.readFile(config, rviz_path)

    mapWidget.frame.load(config)

    mapWidget.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

    mapWidget.frame.setMenuBar( None )
    mapWidget.frame.setStatusBar( None )
    mapWidget.frame.setHideButtonVisibility( False )

    manager = mapWidget.frame.getManager()

    # mapWidget.setGeometry()
    # mapWidget.setFixedSize(500, 400)

    mapLayout = QVBoxLayout()
    mapLayout.addWidget(mapWidget.frame)

    h_layout = QHBoxLayout()

    top_button = QPushButton("Top View")
    # top_button.clicked.connect( _onTopButtonClick )
    h_layout.addWidget( top_button )

    side_button = QPushButton( "Side View" )
    # side_button.clicked.connect( _onSideButtonClick )
    h_layout.addWidget( side_button )
    mapLayout.addLayout( h_layout )

    mapWidget.setLayout(mapLayout)

    return mapWidget, top_button, side_button
