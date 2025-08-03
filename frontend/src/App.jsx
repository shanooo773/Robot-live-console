import { Box, VStack, HStack, Tabs, TabList, TabPanels, Tab, TabPanel } from "@chakra-ui/react";
import CodeEditor from "./components/CodeEditor";
import DebugPanel from "./components/DebugPanel";
import SimulationUploader from "./components/SimulationUploader";

function App() {
  return (
    <Box minH="100vh" bg="#0f0a19" color="gray.500" px={6} py={8}>
      <VStack spacing={6}>
        <DebugPanel />
        
        <Tabs variant="enclosed" colorScheme="blue" width="100%">
          <TabList bg="blackAlpha.300" borderColor="whiteAlpha.300">
            <Tab color="gray.300" _selected={{ color: "white", bg: "blue.600" }}>
              Code Editor
            </Tab>
            <Tab color="gray.300" _selected={{ color: "white", bg: "blue.600" }}>
              Simulation Uploader
            </Tab>
          </TabList>
          
          <TabPanels>
            <TabPanel p={0} pt={6}>
              <CodeEditor />
            </TabPanel>
            <TabPanel p={0} pt={6}>
              <SimulationUploader />
            </TabPanel>
          </TabPanels>
        </Tabs>
      </VStack>
    </Box>
  );
}

export default App;
