import meshio


def convert(source, target):
    mesh = meshio.read(source)
    mesh.write(target)

convert('./models/cylinder.stl','./models/cylinder.vtu')