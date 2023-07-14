import xml.etree.ElementTree as ET


def parse_sdf_file(sdf_file):
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    return root


def convert_sdf_to_urdf(sdf_file, urdf_file):
    sdf_root = parse_sdf_file(sdf_file)

    # URDFのルート要素を作成
    urdf_root = ET.Element("robot")

    origins = {}

    # リンクの変換
    for model in sdf_root.findall("model"):
        for link in model.findall("link"):
            urdf_link = ET.SubElement(urdf_root, "link")
            urdf_link.set("name", link.attrib["name"])

            # link要素の位置と姿勢を抽出してorigin要素として追加
            pose = link.find("pose")
            if pose is not None:
                origins[link.attrib["name"]] = pose.text.split()[0:6]

    # ジョイントの変換
    for model in sdf_root.findall("model"):
        for joint in model.findall("joint"):
            urdf_joint = ET.SubElement(urdf_root, "joint")
            urdf_joint.set("name", joint.attrib["name"])
            urdf_joint.set("type", joint.attrib["type"])

            # child要素とparent要素を抽出して設定
            child = ET.SubElement(urdf_joint, "child")
            child.set("link", joint.find("child").text)

            parent = ET.SubElement(urdf_joint, "parent")
            parent.set("link", joint.find("parent").text)

            if origins.get(joint.find("child").text) is not None:
                origin = ET.SubElement(urdf_joint, "origin")
                origin.set("xyz", " ".join(origins[joint.find("child").text][0:3]))
                origin.set("rpy", " ".join(origins[joint.find("child").text][3:]))

    # URDFファイルに保存
    urdf_tree = ET.ElementTree(urdf_root)
    urdf_tree.write(urdf_file)

    print("Conversion completed successfully.")


sdf_file = "model.sdf"
urdf_file = "output.urdf"
convert_sdf_to_urdf(sdf_file, urdf_file)
