import { Box, VStack, HStack } from "@chakra-ui/react";
import CodeEditor from "./components/CodeEditor";
import DebugPanel from "./components/DebugPanel";

function App() {
  return (
    <Box minH="100vh" bg="#0f0a19" color="gray.500" px={6} py={8}>
      <VStack spacing={6}>
        <DebugPanel />
        <CodeEditor />
      </VStack>
    </Box>
  );
}

export default App;
