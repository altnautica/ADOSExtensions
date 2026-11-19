import { Component, type ReactNode } from "react";

interface ErrorBoundaryProps {
  children: ReactNode;
  /** Rendered when an error is caught. Defaults to nothing. */
  fallback?: ReactNode;
  /** Label for the console warning. */
  label: string;
  /** Clears a caught error when it changes, so the children render again. */
  resetKey?: unknown;
}

/** Catches a render-phase error below it, logs it, and renders the fallback,
 * so one failing viewer never takes down the page around it. */
export class ErrorBoundary extends Component<ErrorBoundaryProps, { hasError: boolean }> {
  state = { hasError: false };

  static getDerivedStateFromError(): { hasError: boolean } {
    return { hasError: true };
  }

  componentDidCatch(error: Error) {
    console.warn(`[${this.props.label}] render failed:`, error.message);
  }

  componentDidUpdate(prev: ErrorBoundaryProps) {
    if (this.state.hasError && !Object.is(prev.resetKey, this.props.resetKey)) {
      this.setState({ hasError: false });
    }
  }

  render() {
    return this.state.hasError ? (this.props.fallback ?? null) : this.props.children;
  }
}
