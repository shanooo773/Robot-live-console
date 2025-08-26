import { extendTheme } from "@chakra-ui/react";

// Custom color palette for robotics theme
const colors = {
  brand: {
    50: "#e6f7ff",
    100: "#bae7ff",
    200: "#7dd3fc",
    300: "#38bdf8",
    400: "#0ea5e9",
    500: "#0284c7",
    600: "#0369a1",
    700: "#075985",
    800: "#0c4a6e",
    900: "#083344",
  },
  robotics: {
    primary: "#00d4ff",
    secondary: "#ff6b6b",
    accent: "#4ecdc4",
    success: "#45b7d1",
    warning: "#f9ca24",
    danger: "#f0932b",
  },
  gradient: {
    primary: "linear(to-r, #667eea, #764ba2)",
    secondary: "linear(to-br, #00d4ff, #0ea5e9)",
    accent: "linear(to-r, #4facfe, #00f2fe)",
    dark: "linear(to-br, #0f0a19, #1a1625, #2d1b69)",
    card: "linear(to-br, rgba(255,255,255,0.1), rgba(255,255,255,0.05))",
  },
  dark: {
    bg: "#0f0a19",
    surface: "#1a1625",
    elevated: "#2d1b69",
    border: "#3d2f7a",
  },
};

// Enhanced typography
const fonts = {
  heading: "'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif",
  body: "'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif",
  mono: "'JetBrains Mono', 'Fira Code', 'Courier New', monospace",
};

// Custom spacing and sizing
const space = {
  px: "1px",
  0.5: "0.125rem",
  1: "0.25rem",
  1.5: "0.375rem",
  2: "0.5rem",
  2.5: "0.625rem",
  3: "0.75rem",
  3.5: "0.875rem",
  4: "1rem",
  5: "1.25rem",
  6: "1.5rem",
  7: "1.75rem",
  8: "2rem",
  9: "2.25rem",
  10: "2.5rem",
  12: "3rem",
  14: "3.5rem",
  16: "4rem",
  20: "5rem",
  24: "6rem",
  28: "7rem",
  32: "8rem",
  36: "9rem",
  40: "10rem",
  44: "11rem",
  48: "12rem",
  52: "13rem",
  56: "14rem",
  60: "15rem",
  64: "16rem",
  72: "18rem",
  80: "20rem",
  96: "24rem",
};

// Enhanced component styles
const components = {
  Button: {
    baseStyle: {
      fontWeight: "semibold",
      borderRadius: "lg",
      _focus: {
        boxShadow: "0 0 0 3px rgba(0, 212, 255, 0.3)",
      },
    },
    variants: {
      solid: {
        bg: "gradient.secondary",
        color: "white",
        _hover: {
          transform: "translateY(-2px)",
          boxShadow: "0 8px 25px rgba(0, 212, 255, 0.3)",
          _disabled: {
            transform: "none",
            boxShadow: "none",
          },
        },
        _active: {
          transform: "translateY(0)",
        },
        transition: "all 0.2s ease-in-out",
      },
      gradient: {
        bgGradient: "gradient.primary",
        color: "white",
        _hover: {
          transform: "translateY(-2px)",
          boxShadow: "0 8px 25px rgba(102, 126, 234, 0.4)",
        },
        _active: {
          transform: "translateY(0)",
        },
        transition: "all 0.2s ease-in-out",
      },
      robotics: {
        bg: "robotics.primary",
        color: "dark.bg",
        _hover: {
          bg: "robotics.accent",
          transform: "translateY(-2px)",
          boxShadow: "0 8px 25px rgba(78, 205, 196, 0.4)",
        },
        _active: {
          transform: "translateY(0)",
        },
        transition: "all 0.2s ease-in-out",
      },
    },
  },
  Card: {
    baseStyle: {
      container: {
        borderRadius: "xl",
        overflow: "hidden",
        transition: "all 0.3s ease-in-out",
      },
    },
    variants: {
      elevated: {
        container: {
          bg: "dark.surface",
          border: "1px solid",
          borderColor: "dark.border",
          boxShadow: "0 4px 20px rgba(0, 0, 0, 0.3)",
          _hover: {
            transform: "translateY(-4px)",
            boxShadow: "0 8px 30px rgba(0, 212, 255, 0.2)",
            borderColor: "robotics.primary",
          },
        },
      },
      gradient: {
        container: {
          bgGradient: "gradient.card",
          backdropFilter: "blur(10px)",
          border: "1px solid",
          borderColor: "whiteAlpha.200",
          _hover: {
            transform: "translateY(-4px)",
            borderColor: "robotics.primary",
            bgGradient: "linear(to-br, rgba(255,255,255,0.15), rgba(255,255,255,0.08))",
          },
        },
      },
    },
    defaultProps: {
      variant: "elevated",
    },
  },
  Input: {
    variants: {
      filled: {
        field: {
          bg: "dark.surface",
          border: "1px solid",
          borderColor: "dark.border",
          _hover: {
            borderColor: "robotics.primary",
            bg: "dark.elevated",
          },
          _focus: {
            borderColor: "robotics.primary",
            boxShadow: "0 0 0 1px rgba(0, 212, 255, 0.3)",
            bg: "dark.elevated",
          },
        },
      },
    },
    defaultProps: {
      variant: "filled",
    },
  },
  Select: {
    variants: {
      filled: {
        field: {
          bg: "dark.surface",
          border: "1px solid",
          borderColor: "dark.border",
          _hover: {
            borderColor: "robotics.primary",
            bg: "dark.elevated",
          },
          _focus: {
            borderColor: "robotics.primary",
            boxShadow: "0 0 0 1px rgba(0, 212, 255, 0.3)",
            bg: "dark.elevated",
          },
        },
      },
    },
    defaultProps: {
      variant: "filled",
    },
  },
  Badge: {
    variants: {
      robotics: {
        bg: "robotics.primary",
        color: "dark.bg",
        fontWeight: "semibold",
      },
      glow: {
        bg: "rgba(0, 212, 255, 0.2)",
        color: "robotics.primary",
        border: "1px solid",
        borderColor: "robotics.primary",
        boxShadow: "0 0 10px rgba(0, 212, 255, 0.3)",
      },
    },
  },
};

// Global styles
const styles = {
  global: {
    body: {
      bg: "dark.bg",
      color: "gray.200",
      fontFamily: "body",
    },
    "*::placeholder": {
      color: "gray.500",
    },
    "*, *::before, &::after": {
      borderColor: "dark.border",
    },
  },
};

// Breakpoints for responsive design
const breakpoints = {
  base: "0em",
  sm: "30em",
  md: "48em",
  lg: "62em",
  xl: "80em",
  "2xl": "96em",
};

const theme = extendTheme({
  config: {
    initialColorMode: "dark",
    useSystemColorMode: false,
  },
  colors,
  fonts,
  space,
  components,
  styles,
  breakpoints,
  shadows: {
    outline: "0 0 0 3px rgba(0, 212, 255, 0.3)",
    glow: "0 0 20px rgba(0, 212, 255, 0.4)",
    "glow-lg": "0 0 40px rgba(0, 212, 255, 0.3)",
  },
});

export default theme;
