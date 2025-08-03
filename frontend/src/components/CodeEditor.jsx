import { useRef, useState } from "react";
import { Box, HStack } from "@chakra-ui/react";
import { Editor } from "@monaco-editor/react";
import RobotSelector from "./RobotSelector";
import { ROBOT_CODE_SNIPPETS } from "../constants";
import VideoPlayer from "./VideoPlayer";

const CodeEditor = () => {
  const editorRef = useRef();
  const [robot, setRobot] = useState("turtlebot");
  const [value, setValue] = useState(ROBOT_CODE_SNIPPETS["turtlebot"]);

  const onMount = (editor) => {
    editorRef.current = editor;
    editor.focus();
  };

  const onSelect = (robotType) => {
    setRobot(robotType);
    setValue(ROBOT_CODE_SNIPPETS[robotType]);
  };

  return (
    <Box>
      <HStack spacing={4}>
        <Box w="50%">
          <RobotSelector robot={robot} onSelect={onSelect} />
          <Editor
            options={{
              minimap: {
                enabled: false,
              },
            }}
            height="75vh"
            theme="vs-dark"
            language="python"
            defaultValue={ROBOT_CODE_SNIPPETS[robot]}
            onMount={onMount}
            value={value}
            onChange={(value) => setValue(value)}
          />
        </Box>
        <VideoPlayer editorRef={editorRef} robot={robot} codeValue={value} />
      </HStack>
    </Box>
  );
};
export default CodeEditor;
